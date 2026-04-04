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
    requires (MeshCompBuilder<Ts, typename Ts::comp_t...> && ...)
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

        auto ci = get_comps_info(&way);
        if (ci.comps.empty()) {
            return false;
        }

        bbox2d bbox;
        std::vector<osm_node> nodes;
        for (auto& nr : way.nodes()) {
            nodes.push_back(nr_to_osm_node(nr, bbox));
        }

        m_entity_db.ways.push_back({ 
            way.id(), 
            std::move(nodes) 
        });

        const char* name = way.tags()["name"];
        m_entity_db.entities.push_back({
            .obj_type = OSM_OBJ_TYPE_WAY,
            .obj_idx = int(m_entity_db.ways.size() - 1),
            .bbox = bbox,
            .comp_flags = ci.comp_flags,
            .way_comp_flags = ci.way_comp_flags,
            .comps = std::move(ci.comps),
            .name = name ? name : ""
        });

        m_entity_db.bbox.extend(bbox);
        return true;
    }

    // Add area.
    bool add_area(const osmium::Area& area)
    {
        if (area.outer_rings().empty()) {
            logWARNING("Rejecting area %lld as it has no outer rings", area.orig_id());
            return false;
        }
        lock_nodes();

        auto ci = get_comps_info(&area);
        if (ci.comps.empty()) {
            return false;
        }

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
            if (ring_orient(ring_span) != desired_orient) {
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

        m_entity_db.areas.push_back({ 
            area.orig_id(), 
            std::move(nodes), 
            std::move(polys) 
        });
        
        const char* name = area.tags()["name"];
        m_entity_db.entities.push_back({
            .obj_type = OSM_OBJ_TYPE_AREA,
            .obj_idx = int(m_entity_db.areas.size() - 1),
            .bbox = bbox,
            .comp_flags = ci.comp_flags,
            .way_comp_flags = ci.way_comp_flags,
            .comps = std::move(ci.comps),
            .name = name ? name : ""
        });

        m_entity_db.bbox.extend(bbox);
        return true;
    }

    // Build meshes.
    bool build(osm_gl_draw_data& out_data)
    {
        if (!finalize_entity_db()) {
            return false;
        }

        bool ret = true;
        std::vector<osm_tri_datad> tri_data;
        std::apply([&](auto&... builders) 
        {
            auto do_build = [&](auto& builder) 
            {
                const char* comp_type = builder.comp_type_name();
                if (!builder.build_all(&m_entity_db, &m_comp_db, tri_data)) {
                    logERROR("Failed to build %s meshes", comp_type);
                    ret = false;
                }
            };
            (do_build(builders), ...);
        }, m_comp_builders);

        if (!ret) {
            return false;
        }

        osm_tri_datad bbox_dd;
        const double ground_offset = -10.0;
        uint32_t vert_startidx = bbox_dd.num_verts();
        bbox_dd.add_vertex(glm::dvec3(m_entity_db.bbox.min.x, m_entity_db.bbox.min.y, ground_offset));
        bbox_dd.add_vertex(glm::dvec3(m_entity_db.bbox.max.x, m_entity_db.bbox.min.y, ground_offset));
        bbox_dd.add_vertex(glm::dvec3(m_entity_db.bbox.max.x, m_entity_db.bbox.max.y, ground_offset));
        bbox_dd.add_vertex(glm::dvec3(m_entity_db.bbox.min.x, m_entity_db.bbox.max.y, ground_offset));
        bbox_dd.add_triangle_w_offset({ 0, 1, 2 }, vert_startidx, TRI_TYPE_GROUND);
        bbox_dd.add_triangle_w_offset({ 0, 2, 3 }, vert_startidx, TRI_TYPE_GROUND);

        tri_data.push_back(std::move(bbox_dd));

        log_func("Computing normals", [&]()
        {
            osm_tri_dataf df;
            for (auto& dd : tri_data) {
                df.add_tridata(dd);
            }

            std::vector<glm::fvec3> tri_normals(df.tris().size());
            std::vector<std::vector<uint32_t>> vert2tris(df.verts().size());

            for (uint32_t itri = 0; itri < uint32_t(df.tris().size()); ++itri)
            {
                auto& tri = df.tris()[itri];
                auto& indices = tri.vert_idxs;

                auto v01 = df.verts()[indices[1]] - df.verts()[indices[0]];
                auto v02 = df.verts()[indices[2]] - df.verts()[indices[0]];
                auto normal = glm::normalize(glm::cross(v01, v02));
                tri_normals[itri] = normal;

                for (int i = 0; i < 3; ++i) {
                    vert2tris[indices[i]].push_back(itri);
                }
            }

            const float SMOOTH_THRESH = std::cos(glm::radians(20.0f));

            std::vector<glm::fvec3> tri_corner_normals(df.tris().size() * 3);
            for (uint32_t itri = 0; itri < uint32_t(df.tris().size()); ++itri)
            {
                auto tri_type = df.tris()[itri].type;
                auto& indices = df.tris()[itri].vert_idxs;

                for (int i = 0; i < 3; ++i)
                {
                    auto& corner_normal = tri_corner_normals[3 * itri + i];
                    for (uint32_t iadjface : vert2tris[indices[i]])
                    {
                        auto adj_tri_type = df.tris()[iadjface].type;
                        auto& adj_normal = tri_normals[iadjface];

                        if (adj_tri_type == tri_type &&
                            glm::dot(adj_normal, tri_normals[itri]) >= SMOOTH_THRESH) {
                            corner_normal += adj_normal;
                        }
                    }
                    corner_normal = glm::normalize(corner_normal);
                }
            }

            osm_gl_draw_data result;
            for (uint32_t itri = 0; itri < uint32_t(df.tris().size()); ++itri)
            {
                auto tri_type = df.tris()[itri].type;
                auto& og_indices = df.tris()[itri].vert_idxs;

                uint32_t verts_idx = uint32_t(result.verts.size());
                for (int i = 0; i < 3; ++i)
                {
                    auto& vert = df.verts()[og_indices[i]];
                    auto& normal = tri_corner_normals[3 * itri + i];
                    result.verts.push_back({ vert.x, vert.y, vert.z, normal.x, normal.y, normal.z });
                }
                result.tris[tri_type].push_back({ verts_idx, verts_idx + 1, verts_idx + 2 });
            }

            out_data = std::move(result);
        });

        return true;
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

    struct obj_comps_info
    {
        mesh_entity::comp_info_vec_t comps;
        mesh_entity::comp_flags_t comp_flags;
        mesh_entity::way_comp_flags_t way_comp_flags;
    };
    obj_comps_info get_comps_info(const osmium::OSMObject* obj)
    {
        obj_comps_info ret;

        std::apply([&](auto&... builders) {
            (builders.add_comp(int(m_entity_db.entities.size()), obj, &m_comp_db, ret.comps), ...);
        }, m_comp_builders);

        for (const auto& comp : ret.comps) 
        {
            assert_msg(!ret.comp_flags[comp.type], 
                "Object %lld has multiple comps of type %d", obj->id(), comp.type);
            
            ret.comp_flags[comp.type] = true;

            if (obj->type() == osmium::item_type::way) {
                switch (comp.type) {
                case COMP_TYPE_HIGHWAY: ret.way_comp_flags[WAY_COMP_TYPE_HIGHWAY] = true;  break;
                case COMP_TYPE_WATER:   ret.way_comp_flags[WAY_COMP_TYPE_WATERWAY] = true; break;
                default: break;
                }
            }
        }
        return ret;
    }

    bool finalize_entity_db()
    {
        //m_entity_db.center = m_center;
        double center_lat = osmium::geom::detail::y_to_lat(m_center.y);
        m_entity_db.ht_scale = 1.0 / std::cos(glm::radians(center_lat));

        log_func("Building spatial index", [&]() 
        {
            buffer<const mesh_entity*> obj_ptrs(m_entity_db.entities.size(), buffer_overwrite);
            for (size_t i = 0; i < m_entity_db.entities.size(); ++i) {
                obj_ptrs.ptr[i] = &m_entity_db.entities[i];
            }
            m_entity_db.entity_tree = { aabb_tree_unsafe_ctor_t{}, obj_ptrs.span() };
        }, "Entity DB");

        log_func("Building way network", [&]() 
        {
            for (const auto& way : m_entity_db.entities)
            {
                if (way.obj_type != OSM_OBJ_TYPE_WAY) {
                    continue;
                }
                auto& way_osm = m_entity_db.get<osm_way>(way.obj_idx);

                for (size_t i = 0; i < way_osm.nodes.size(); ++i)
                {
                    auto* prev_way_node = (i == 0) ? nullptr : &way_osm.nodes[i - 1];
                    auto* cur_way_node = &way_osm.nodes[i];
                    auto* next_way_node = (i == way_osm.nodes.size() - 1) ? nullptr : &way_osm.nodes[i + 1];

                    auto nodeitr = m_entity_db.way_net.get_or_add_node(cur_way_node->id, cur_way_node->vert);

                    auto& adj_node_ids = nodeitr->second.adj_node_ids;
                    if (prev_way_node) {
                        adj_node_ids.insert(prev_way_node->id);
                    }
                    if (next_way_node) {
                        adj_node_ids.insert(next_way_node->id);
                        m_entity_db.way_net.add_edge({ nodeitr->first, next_way_node->id }, &way);
                    }
                }
            }
        }, "Entity DB");

        return true;
    }

private:
    mesh_entity_db m_entity_db;
    mesh_comp_db<typename Ts::comp_t...> m_comp_db;
    std::tuple<Ts...> m_comp_builders;

    glm::dvec2 m_center{ 0.0, 0.0 };
    std::size_t m_num_nodes = 0;
    bool m_nodes_locked = false;
};

#endif
