
#include <cmath>
#include <osmium/geom/mercator_projection.hpp>

#include "containers/aabb_tree.hpp"
#include "mesh_builder.hpp"


// Parse number with null check
template <typename T>
static bool parse_num_if_exists(std::string_view str, T& val)
{
    return str && parse_num(str, val);
}

bool mesh_builder::add_node(const osmium::Node& node)
{
    if (m_nodes_locked) {
        logERROR("All nodes must be added before ways/areas.");
        return false;
    }
    // note: averaging lat/lon here to avoid projecting every point.
    // The error from this is negligible unless 
    // the OSM data is very large or near the poles.
    auto loc = node.location();
    m_center += glm::dvec2(loc.lon(), loc.lat());
    m_num_nodes++;

    return true;
}

void mesh_builder::lock_nodes()
{
    if (!m_nodes_locked) {
        m_center /= m_num_nodes;
        auto center_loc = osmium::Location(m_center.x, m_center.y);
        auto center_proj = osmium::geom::MercatorProjection{}(center_loc);
        m_center = glm::dvec2(center_proj.x, center_proj.y);
        m_nodes_locked = true;
    }
}

osm_node mesh_builder::nr_to_osm_node(const osmium::NodeRef& nr)
{
    auto proj = osmium::geom::MercatorProjection{}(nr.location());
    auto proj_glm = glm::dvec2(proj.x, proj.y);
    return { nr.ref(), proj_glm - m_center };
}

osm_node mesh_builder::nr_to_osm_node(const osmium::NodeRef& nr, bbox2d& bbox)
{
    auto node = nr_to_osm_node(nr);
    bbox.extend(node.vert);
    return node;
}

bool mesh_builder::add_way(const osmium::Way& way)
{
    lock_nodes();

    mesh_object::comp_vec_t comps;
    add_street_comp(&way, comps);

    bbox2d bbox = bbox2d::empty();
    if (!comps.empty()) 
    {
        std::vector<osm_node> nodes(way.nodes().size());
        for (auto& nr : way.nodes()) {
            nodes.push_back(nr_to_osm_node(nr, bbox));
        }

        m_ways.push_back({ way.id(), std::move(nodes) });

        const char* name = way.tags()["name"];
        m_objects.push_back({
            .type = mesh_object::OBJ_TYPE_WAY,
            .osm_obj_idx = int(m_ways.size() - 1),
            .bbox = bbox,
            .comps = std::move(comps),
            .name = name ? name : ""
        });

        for (auto& comp : m_objects.back().comps) {
            comp.parent_obj_idx = int(m_objects.size() - 1);
        }
        return true;
    }

    return false;
}

bool mesh_builder::add_area(const osmium::Area& area)
{
    lock_nodes();

    if (area.outer_rings().empty()) {
        logWARNING("Rejecting area %lld as it has no outer rings", area.orig_id());
        return false;
    }

    mesh_object::comp_vec_t comps;
    add_street_comp(&area, comps);
    add_bldg_comp(&area, comps);

    if (!comps.empty()) 
    {
        std::vector<osm_node> nodes;
        std::vector<std::tuple<int, int, int>> ring_bounds;

        auto add_ring = [&](int poly_idx, 
            const auto& ring, orient_t desired_orient, auto add_node)
        {
            assert(ring.is_closed() && ring.size() >= 3);

            int start_idx = int(nodes.size());
            int ring_size = int(ring.size() - 1);

            for (int i = 0; i < ring_size; ++i) {
                add_node(ring[i]);
            }
            auto ring_span = std::span(&nodes[start_idx], ring_size);
            if (path_orient(ring_span) != desired_orient) {
                std::reverse(ring_span.begin(), ring_span.end());
            }
            ring_bounds.push_back({ poly_idx, start_idx, ring_size });
        };

        int poly_idx = 0;
        bbox2d bbox = bbox2d::empty();
        for (const auto& outer_ring : area.outer_rings()) 
        {
            add_ring(poly_idx, outer_ring, ORIENT_CCW, [&](const auto& nr) { 
                return nr_to_osm_node(nr, bbox); 
            });
            for (const auto& inner_ring : area.inner_rings(outer_ring)) {
                add_ring(poly_idx, inner_ring, ORIENT_CW, [&](const auto& nr) { 
                    return nr_to_osm_node(nr); 
                });
            }
            poly_idx++;
        }

        osm_area::multipoly_t polys(area.outer_rings().size());
        for (const auto& [poly_idx, start_idx, ring_size] : ring_bounds) {
            polys[poly_idx].push_back({ &nodes[start_idx], ring_size });
        }

        m_areas.push_back({ area.orig_id(), std::move(polys) });

        const char* name = area.tags()["name"];
        m_objects.push_back({
            .type = mesh_object::OBJ_TYPE_AREA,
            .osm_obj_idx = int(m_areas.size() - 1),
            .bbox = bbox,
            .comps = std::move(comps),
            .name = name ? name : ""
        });

        for (auto& comp : m_objects.back().comps) {
            comp.parent_obj_idx = int(m_objects.size() - 1);
        }
        return true;
    }
}

void mesh_builder::build()
{
}

std::vector<draw_datad> mesh_builder::get_draw_data()
{
    aabb_tree2d<building*> bldg_tree;

    std::vector<draw_datad> ret;
    gen_building_drawdata(ret, &bldg_tree);
    gen_street_drawdata(ret, &bldg_tree);

    logMESSAGE("-----------------------------------------------\n");

    // center vertices to reduce precision issues
    size_t ivert = 0;
    glm::dvec3 meanpos(0.0);

    for (draw_datad& dd : ret) {
        for (uint32_t i = 0; i < dd.num_verts(); ++i) {
            // incremental mean to avoid overflow
            meanpos += (dd.get_vertex(i) - meanpos) / double(ivert + 1);
            ivert++;
        }
    }
    for (draw_datad& dd : ret) {
        for (size_t i = 0; i < dd.verts.size(); i += 3) {
            dd.verts[i + 0] -= meanpos.x;
            dd.verts[i + 1] -= meanpos.y;
            dd.verts[i + 2] -= meanpos.z;
        }
    }

    //// add ground plane
    //draw_datad ground_dd;
    //ground_dd.color = { 0.2f, 0.2f, 0.2f, 1.0f };
    //ground_dd.name = "ground plane";
    //ground_dd.verts = {
    //    bbox.min.x, bbox.min.y, bbox.min.z - 0.1,
    //    bbox.max.x, bbox.min.y, bbox.min.z - 0.1,
    //    bbox.max.x, bbox.max.y, bbox.min.z - 0.1,
    //    bbox.min.x, bbox.max.y, bbox.min.z - 0.1
    //};
    //ground_dd.tri_indices = { 0, 2, 1, 0, 3, 2 };
    //
    //ret.push_back(std::move(ground_dd));

    return ret;
}

