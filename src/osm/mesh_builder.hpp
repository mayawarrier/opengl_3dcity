#ifndef OSM_MESH_HPP
#define OSM_MESH_HPP

#include <vector>

#include <osmium/osm/types.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/relations/relations_database.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include "comp_builders/types.hpp"

#include "common.hpp"


// Converts OSM data into triangles for rendering.
template <class... Ts>
    requires (MeshCompBuilder<Ts, typename Ts::comp_type...> && ...)
class osm_mesh_builder
{
public:
    // Add node. All nodes must be added before all ways/areas.
    bool add_node(const osmium::Node& node)
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

    // todo: there are closed ways that are not areas!
    // (and sometimes they can be both, according to different tags).
    // https://wiki.openstreetmap.org/wiki/Area

    // Add linear way. Closed ways should use add_area().
    bool add_way(const osmium::Way& way)
    {
        assert_msg(!way.is_closed(), "Closed ways must be added as areas");
        lock_nodes();

        osm_mesh_object::comp_info_vec_t comps;
        osm_mesh_object::comp_flags_t comp_flags;
        get_comps(&way, comps, comp_flags);

        if (!comps.empty())
        {
            bbox2d bbox;
            std::vector<osm_node> nodes;
            for (auto& nr : way.nodes()) {
                nodes.push_back(nr_to_osm_node(nr, bbox));
            }

            m_obj_db.ways.push_back({ 
                way.id(), 
                std::move(nodes) 
            });

            const char* name = way.tags()["name"];
            m_obj_db.objects.push_back({
                .type = OSM_OBJ_TYPE_WAY,
                .osm_obj_idx = int(m_obj_db.ways.size() - 1),
                .bbox = bbox,
                .comp_flags = comp_flags,
                .comps = std::move(comps),
                .name = name ? name : ""
            });

            m_obj_db.objs_bbox.extend(bbox);
            return true;
        }

        return false;
    }

    // Add area.
    bool add_area(const osmium::Area& area)
    {
        if (area.outer_rings().empty()) {
            logWARNING("Rejecting area %lld as it has no outer rings", area.orig_id());
            return false;
        }
        lock_nodes();

        osm_mesh_object::comp_info_vec_t comps;
        osm_mesh_object::comp_flags_t comp_flags;
        get_comps(&area, comps, comp_flags);

        if (!comps.empty()) 
        {
            std::vector<osm_node> nodes;
            std::vector<std::tuple<int, int, int>> ring_bounds;

            auto add_ring = [&](int poly_idx, 
                const auto& ring, orient_t desired_orient, auto get_node)
            {
                assert(ring.is_closed() && ring.size() >= 3);

                int start_idx = int(nodes.size());
                int ring_size = int(ring.size()) - 1;

                for (int i = 0; i < ring_size; ++i) {
                    nodes.push_back(get_node(ring[i]));
                }
                auto ring_span = std::span(&nodes[start_idx], ring_size);
                if (path_orient(ring_span) != desired_orient) {
                    std::reverse(ring_span.begin(), ring_span.end());
                }
                ring_bounds.push_back({ poly_idx, start_idx, ring_size });
            };

            bbox2d bbox;
            int poly_idx = 0;          
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
                polys[poly_idx].push_back({ &nodes[start_idx], size_t(ring_size) });
            }

            m_obj_db.areas.push_back({ 
                area.orig_id(), 
                std::move(nodes), 
                std::move(polys) 
            });
            
            const char* name = area.tags()["name"];
            m_obj_db.objects.push_back({
                .type = OSM_OBJ_TYPE_AREA,
                .osm_obj_idx = int(m_obj_db.areas.size() - 1),
                .bbox = bbox,
                .comp_flags = comp_flags,
                .comps = std::move(comps),
                .name = name ? name : ""
            });

            m_obj_db.objs_bbox.extend(bbox);
            return true;
        }
    }

    // Build meshes.
    bool build(std::vector<draw_datad>& out_drawdata)
    {
        buffer<const osm_mesh_object*> obj_ptrs(m_obj_db.objects.size(), buffer_overwrite);
        for (size_t i = 0; i < m_obj_db.objects.size(); ++i) {
            obj_ptrs.ptr[i] = &m_obj_db.objects[i];
        }
        m_obj_db.obj_tree = { aabb_tree_unsafe_ctor_t{}, obj_ptrs.span() };

        bool ret = true;
        std::apply([&](auto&... builders) 
        {
            auto do_build = [&](auto& builder) 
            {
                const char* comp_type = builder.comp_type_name();
                logMESSAGE("-----------------------------------------------");
                logMESSAGE("Generating %s meshes...", comp_type);

                if (!builder.build_all(&m_obj_db, &m_comp_db, out_drawdata)) {
                    logERROR("Failed to build %s meshes", comp_type);
                    ret = false;
                }
            };
            (do_build(builders), ...);
        }, 
            m_comp_builders);

        logMESSAGE("-----------------------------------------------\n");
        return ret;
    }

private:
    void lock_nodes()
    {
        if (!m_nodes_locked) {
            m_center /= m_num_nodes;
            auto center_loc = osmium::Location(m_center.x, m_center.y);
            auto center_proj = osmium::geom::MercatorProjection{}(center_loc);
            m_center = glm::dvec2(center_proj.x, center_proj.y);
            m_nodes_locked = true;
        }
    }

    osm_node nr_to_osm_node(const osmium::NodeRef& nr)
    {
        auto proj = osmium::geom::MercatorProjection{}(nr.location());
        auto proj_glm = glm::dvec2(proj.x, proj.y);
        return { nr.ref(), proj_glm - m_center };
    }

    osm_node nr_to_osm_node(const osmium::NodeRef& nr, bbox2d& bbox)
    {
        auto node = nr_to_osm_node(nr);
        bbox.extend(node.vert);
        return node;
    }

    bool get_comps(const osmium::OSMObject* obj, 
        osm_mesh_object::comp_info_vec_t& out_comps, osm_mesh_object::comp_flags_t& out_flags)
    {
        std::apply([&](auto&... builders) {
            (builders.add_comp(int(m_obj_db.objects.size()), obj, &m_comp_db, out_comps), ...);
        }, m_comp_builders);

        for (const auto& comp : out_comps) {
            if (out_flags.test(comp.type)) {
                assert_msg(false, "Object %lld has multiple comps of type %d", obj->id(), comp.type);
                return false;
            }
            out_flags.set(comp.type);
        }
    }

private:
    osm_mesh_object_db m_obj_db;
    osm_mesh_comp_db<typename Ts::comp_type...> m_comp_db;
    std::tuple<Ts...> m_comp_builders;

    glm::dvec2 m_center{ 0.0, 0.0 };
    std::size_t m_num_nodes = 0;
    bool m_nodes_locked = false;
};

#endif
