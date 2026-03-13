#ifndef OSM_MESH_BUILDINGS_IMPL_HPP
#define OSM_MESH_BUILDINGS_IMPL_HPP

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <boost/container/flat_set.hpp>

#include "../geom.hpp"
#include "../mesh_builder.hpp"

#include "buildings.hpp"



bool bldg_comp_builder::do_build_all(const osm_mesh_object_db* obj_db, const std::vector<bldg_comp>& bldgs, std::vector<draw_datad>& out_drawdata)
{
    return true;
}

void bldg_comp_builder::set_bldg_heights(const osmium::TagList& tags, bldg_comp& comp)
{
    int levels, min_level;
    double height, min_height;

    bool has_levels =    parse_num(tags["building:levels"], levels);
    bool has_minlevel =  parse_num(tags["building:min_level"], min_level);
    bool has_height =    parse_num(tags["height"], height);
    bool has_minheight = parse_num(tags["min_height"], min_height);

    comp.bldg_ht_top = (has_height && height > 0) ? height : 
                       (has_levels && levels > 0) ? (3.0 * levels) : 
                       -1.0;
    comp.bldg_ht_btm = (has_minheight && min_height >= 0) ? min_height : 
                       (has_minlevel && min_level >= 0) ? (3.0 * min_level) :
                       -1.0;

    bool valid_range = comp.bldg_ht_top > comp.bldg_ht_btm;
    bool has_ht_btm = comp.bldg_ht_btm >= 0.0 && valid_range;
    bool has_ht_top = comp.bldg_ht_top > 0.0 && valid_range;

    if (!has_ht_btm) { comp.bldg_ht_btm = 0.0; }
    if (!has_ht_top) { comp.bldg_ht_top = comp.bldg_ht_btm + 3.0; }

    comp.roof_ht_top = -1.0; // not supported yet
}

//static inline void mesh_add_triangle(draw_datad& mesh, glm::u32vec3 indices, bool reverse_winding)
//{
//    if (reverse_winding) {
//        mesh.add_triangle(indices[0], indices[2], indices[1]);
//    } else {
//        mesh.add_triangle(indices[0], indices[1], indices[2]);
//    }
//}
//
//static uint32_t mesh_add_polygon(draw_datad& mesh, std::span<const glm::dvec2> verts,
//    std::span<const uint32_t> indices, double height, bool reverse_winding = false)
//{
//    uint32_t vert_startidx = uint32_t(mesh.num_verts());
//    for (const auto& vert : verts) {
//        mesh.add_vertex(vert.x, vert.y, height);
//    }
//
//    for (size_t i = 0; i < indices.size(); i += 3)
//    {
//        uint32_t idx0 = indices[i] + vert_startidx;
//        uint32_t idx1 = indices[i + 1] + vert_startidx;
//        uint32_t idx2 = indices[i + 2] + vert_startidx;
//
//        mesh_add_triangle(mesh, { idx0, idx1, idx2 }, reverse_winding);
//    }
//    return vert_startidx;
//}
//
//// \param verts_ccw True if the vertices are in CCW order.
//static void mesh_add_sides(draw_datad& mesh, uint32_t bot_verts_idx, 
//    uint32_t top_verts_idx, uint32_t num_verts, bool verts_ccw = true)
//{
//    for (uint32_t icur = 0; icur < num_verts; ++icur)
//    {
//        uint32_t inext = (icur + 1) % num_verts;
//        uint32_t quad[4] = {
//            bot_verts_idx + icur,
//            bot_verts_idx + inext,
//            top_verts_idx + icur,
//            top_verts_idx + inext,
//        };
//        mesh_add_triangle(mesh, { quad[0], quad[2], quad[3] }, verts_ccw);
//        mesh_add_triangle(mesh, { quad[0], quad[3], quad[1] }, verts_ccw);
//    }
//}
//
//bool mesh_builder::get_building_part_mesh(draw_datad& mesh, const building_part& part)
//{
//    std::vector<uint32_t> tri_indices;
//    if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
//        tri_indices = polygon_triangulate(m_bldg_areas[part.id].rings);
//    } else {
//        auto vert_span = std::span<const glm::dvec2>(part.verts);
//        tri_indices = polygon_triangulate(std::span(&vert_span, 1));
//    }
//
//    assert(check_triangles_oriented(part.verts, tri_indices));
//
//    uint32_t bot_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_btm);
//    uint32_t top_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_top, true);
//
//    if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] 
//    {
//        auto& rings = m_bldg_areas[part.id].rings;
//
//        auto& outer_ring = rings[0];
//        mesh_add_sides(mesh, bot_verts_idx, top_verts_idx, uint32_t(outer_ring.size()));
//
//        uint32_t vert_offset = uint32_t(outer_ring.size());
//        for (size_t iring = 1; iring < rings.size(); ++iring) 
//        {
//            auto& inner_ring = rings[iring];
//            uint32_t inner_ring_size = uint32_t(inner_ring.size());
//            uint32_t inner_bot_idx = bot_verts_idx + vert_offset;
//            uint32_t inner_top_idx = top_verts_idx + vert_offset;
//
//            mesh_add_sides(mesh, inner_bot_idx, inner_top_idx, inner_ring_size, false);
//            vert_offset += inner_ring_size;
//        }
//    }
//    else {
//        mesh_add_sides(mesh, bot_verts_idx, top_verts_idx, uint32_t(part.verts.size()));
//    }
//    return true;
//}
//
//template <class ...TComps>
//bool bldg_comp_builder::build_all(const osm_mesh_object_db* obj_db,
//    osm_mesh_comp_db<TComps...>* comp_db, std::vector<draw_datad>& out_drawdata)
//{
//    size_t num_verts = 0, num_tris = 0;
//    auto add_drawdata = [&](draw_datad&& dd) {
//        num_verts += dd.num_verts();
//        num_tris += dd.num_tris();
//        drawdata.push_back(std::move(dd));
//    };
//
//    logMESSAGE("%zu buildings, %zu parts, %zu areas", 
//        m_buildings.size(), m_building_parts.size(), m_bldg_areas.size());
//
//    auto tbegin = clk::now();
//
//    // Build AABB tree for fast intersection queries
//    auto& bldg_tree = *bldg_tree_ptr;
//    timeit("Building AABB tree", [&]()
//    {
//        buffer<building*> tree_objects(m_buildings.size(), buffer_overwrite);
//        for (size_t i = 0; i < m_buildings.size(); ++i) {
//            tree_objects.ptr[i] = &m_buildings[i];
//        }
//        bldg_tree = aabb_tree2d<building*>::create_unsafe(tree_objects.span());
//    });
//
//    // Map parts/areas to buildings
//    std::vector<building_part*> unmapped_parts;  
//    timeit("Mapping parts to buildings", [&]()
//    {
//        for (auto& part : m_building_parts)
//        {
//            auto inter_bldgs = bldg_tree.query_bbox_all(part.bbox);
//            if (inter_bldgs.empty()) {
//                logDEBUG(LOG_WARNING, "Part %lld tree query returned nothing", part.id);
//                unmapped_parts.push_back(&part);
//                continue;
//            }
//
//            polygon_cspan part_poly;
//            std::span<const glm::dvec2> part_verts;
//
//            if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
//                part_poly = m_bldg_areas[part.id].rings;
//            } else {
//                part_verts = part.verts;
//                part_poly = std::span(&part_verts, 1);
//            }
//
//            building* mapped_bldg = nullptr;
//            for (size_t icand = 0; icand < inter_bldgs.size(); ++icand) 
//            {
//                polygon_cspan bldg_poly;
//                std::span<const glm::dvec2> bldg_verts;
//
//                if (inter_bldgs[icand]->base.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
//                    bldg_poly = m_bldg_areas[inter_bldgs[icand]->base.id].rings;
//                } else {
//                    bldg_verts = inter_bldgs[icand]->base.verts;
//                    bldg_poly = std::span(&bldg_verts, 1);
//                }
//            
//                if (polygon_covered_by(part_poly, bldg_poly)) {
//                    mapped_bldg = inter_bldgs[icand];
//                    break;
//                }
//            }
//            if (mapped_bldg) {
//                mapped_bldg->parts.push_back(&part);
//            } else {
//                logDEBUG(LOG_MESSAGE, "Could not map part %lld to a building", part.id);
//                logDEBUG(LOG_MESSAGE, "Tried candidates:");
//                for (auto& bldg : inter_bldgs) {
//                    logDEBUG(LOG_MESSAGE, "  Building %lld", bldg->base.id);
//                }
//                unmapped_parts.push_back(&part);
//            }
//        }
//
//        if (!unmapped_parts.empty()) {
//            logWARNING("%zu/%zu part(s) could not be mapped to a building", 
//                unmapped_parts.size(), m_building_parts.size());
//        }
//    });
//
//    const glm::vec4 building_color(0.85f, 0.75f, 0.65f, 1.0f);
//
//    // Build meshes from the parts
//    timeit("Building meshes", [&]()
//    {
//        for (auto& building : m_buildings)
//        {
//            if (building.name.empty()) {
//                logDEBUG(LOG_MESSAGE, "Adding building %lld", building.base.id);
//            } else {
//                logDEBUG(LOG_MESSAGE, "Adding building %lld (%s)", building.base.id, building.name.c_str());
//            }
//
//            if (building.parts.empty() || building.base.has_ht_top)
//            {
//                draw_datad dd;
//                if (get_building_part_mesh(dd, building.base)) {
//                    dd.color = building_color;
//                    dd.name = building.name.empty() ? "bldg " + std::to_string(building.base.id) : building.name;
//                    add_drawdata(std::move(dd));
//                }
//            }
//            for (auto* part : building.parts)
//            {
//                logDEBUG(LOG_MESSAGE, "   Adding part %lld", part->id);
//
//                draw_datad dd;
//                if (get_building_part_mesh(dd, *part)) {
//                    dd.color = building_color;
//                    dd.name = "part " + std::to_string(part->id);
//                    add_drawdata(std::move(dd));
//                }
//            }
//        }
//
//        for (auto* part : unmapped_parts)
//        {
//            draw_datad dd;
//            if (get_building_part_mesh(dd, *part)) {
//                dd.color = building_color;
//                dd.name = "part " + std::to_string(part->id);
//                add_drawdata(std::move(dd));
//            }
//        }
//    });
//
//    auto tend = clk::now();
//    logMESSAGE("Generated %u tris and %u vertices in %s",
//        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());
//
//    return true;
//}

#endif