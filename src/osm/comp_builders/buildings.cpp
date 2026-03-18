#ifndef OSM_MESH_BUILDINGS_IMPL_HPP
#define OSM_MESH_BUILDINGS_IMPL_HPP

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <boost/container/flat_set.hpp>

#include "../geom/geom.hpp"
#include "../mesh_builder.hpp"

#include "buildings.hpp"


void bldg_comp_builder::set_bldg_heights(const osmium::TagList& tags, bldg_comp& comp)
{
    int levels, min_level;
    double height, min_height;

    bool has_levels =    parse_num(tags["building:levels"], levels);
    bool has_minlevel =  parse_num(tags["building:min_level"], min_level);
    bool has_height =    parse_num(tags["height"], height);
    bool has_minheight = parse_num(tags["min_height"], min_height);

    comp.ht_top = (has_height && height > 0) ? height : 
                  (has_levels && levels > 0) ? (3.0 * levels) : 
                  -1.0;
    comp.ht_btm = (has_minheight && min_height >= 0) ? min_height : 
                  (has_minlevel && min_level >= 0) ? (3.0 * min_level) :
                  -1.0;

    bool valid_range = comp.ht_top > comp.ht_btm;
    comp.has_ht_btm = comp.ht_btm >= 0.0 && valid_range;
    comp.has_ht_top = comp.ht_top > 0.0 && valid_range;

    if (!comp.has_ht_btm) { comp.ht_btm = 0.0; }
    if (!comp.has_ht_top) { comp.ht_top = comp.ht_btm + 3.0; }

    comp.roof_ht = -1.0; // not supported yet
}

// Maps building parts to their parent buildings.
class bldg_part_mapper
{
private:
    using comp_mapping_t = types::unord_flat_map<const bldg_comp*, std::vector<const bldg_comp*>>;

    struct part_map_log
    {
        osmium::object_id_type part_id;
        std::vector<osmium::object_id_type> inter_bldg_ids;
        std::vector<osmium::object_id_type> covered_bldg_ids;
        std::vector<osmium::object_id_type> pass2_bldg_ids;
        std::vector<osmium::object_id_type> pass3_bldg_ids;
    };

public:
    using bldg2part_map_type = comp_mapping_t;

public:
    bldg_part_mapper(const osm_mesh_object_db* obj_db,
        const std::vector<bldg_comp>& all_comps,
        const std::vector<const bldg_comp*>& part_comps) :
        m_obj_db(obj_db),
        m_all_comps(all_comps),
        m_part_comps(part_comps)
    {}

    void do_mapping()
    {
        types::unord_flat_map<const bldg_comp*, part_map_log> mapping_logs;

        for (auto* part_comp : m_part_comps)
        {
            auto& part = m_obj_db->get<osm_mesh_object>(part_comp->mesh_obj_idx);
            auto& part_osm = m_obj_db->get<osm_area>(part.osm_obj_idx);
            
            auto& log = mapping_logs[part_comp];
            log.part_id = part_osm.id;

            osm_mesh_object::comp_flags_t search_flags;
            search_flags.set(COMP_TYPE_BUILDING);

            // Get nearby buildings.
            auto cand_bldgs = m_obj_db->obj_tree.query_intersecting_bboxes(part.bbox, search_flags);
            if (cand_bldgs.empty()) {
                logDEBUG(LOG_WARNING, "Part %lld tree query returned nothing", part_osm.id);
                m_unmapped_parts.push_back(part_comp);
                continue;
            }
            for (const auto* bldg : cand_bldgs) {
                log.inter_bldg_ids.push_back(m_obj_db->obj_osm_id(*bldg));
            }

            // Remove buildings whose polygons do not fully contain the part.
            std::erase_if(cand_bldgs, [&](const auto* bldg) {
                auto& bldg_osm = m_obj_db->get<osm_area>(bldg->osm_obj_idx);
                return !multipoly_covered_by(part_osm.polys, bldg_osm.polys, 1e-2);
            });
            for (const auto* bldg : cand_bldgs) {
                log.covered_bldg_ids.push_back(m_obj_db->obj_osm_id(*bldg));
            }
            if (cand_bldgs.empty()) {               
                log_failure(log);
                m_unmapped_parts.push_back(part_comp);            
                continue;
            }

            // todo: in many cases it seems like the parts do not encompass the full building polygon,
            // and the outline must be drawn in that case.

            if (cand_bldgs.size() == 1) {
                m_bldg2part_map[get_comp_ptr(cand_bldgs[0])].push_back(part_comp);
            } else {
                logDEBUG(LOG_WARNING, "Part %lld is covered by multiple buildings", part_osm.id);
                map_part_pass2(part_comp, cand_bldgs, log);
            }
        }

        for (auto& [part_comp, bldg_comps] : m_pass3_part2bldg)
        {
            if (!map_part_pass3(part_comp, bldg_comps, mapping_logs[part_comp])) {
                log_failure(mapping_logs[part_comp]);
                m_unmapped_parts.push_back(part_comp);
            }
        }
    }

    const std::vector<const bldg_comp*>& unmapped_parts() const {
        return m_unmapped_parts;
    }

    const bldg2part_map_type& bldg2parts() const {
        return m_bldg2part_map;
    }

private:
    const bldg_comp* get_comp_ptr(const osm_mesh_object* obj) const {
        return &m_all_comps[obj->get_comp_idx(COMP_TYPE_BUILDING)];
    }

    void log_failure(const part_map_log& log)
    {
        logDEBUG(LOG_WARNING, "Could not map part %lld to a building", log.part_id);

        std::string logstr;
        if (!log.inter_bldg_ids.empty()) {
            logstr += "Intersected: ";
            logstr += str_join(log.inter_bldg_ids, ", ");
            logstr += "\n";
        }
        if (!log.covered_bldg_ids.empty()) {
            logstr += "Covered: ";
            logstr += str_join(log.covered_bldg_ids, ", ");
            logstr += "\n";
        }
        if (!log.pass2_bldg_ids.empty()) {
            logstr += "After pass 2: ";
            logstr += str_join(log.pass2_bldg_ids, ", ");
            logstr += "\n";
        }
        if (!log.pass3_bldg_ids.empty()) {
            logstr += "After pass 3: ";
            logstr += str_join(log.pass3_bldg_ids, ", ");
            logstr += "\n";
        }
        
        logDEBUG(LOG_MESSAGE, logstr.c_str());
    }

    bool map_part_pass2(const bldg_comp* part_comp,
         std::vector<const osm_mesh_object*>& cand_bldgs, part_map_log& log)
    {
        auto handle_in_pass3 = [&](const osm_mesh_object* bldg) 
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            m_pass3_bldg2part[bldg_comp].push_back(part_comp);
            m_pass3_part2bldg[part_comp].push_back(bldg_comp);
            log.pass2_bldg_ids.push_back(m_obj_db->obj_osm_id(*bldg));
        };

        auto add_mapping = [&](const osm_mesh_object* bldg) 
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            m_bldg2part_map[bldg_comp].push_back(part_comp);
            log.pass2_bldg_ids.push_back(m_obj_db->obj_osm_id(*bldg));
        };

        if (!part_comp->has_ht_top) {
            // part has no height info, defer to pass 3
            for (const auto* bldg : cand_bldgs) {
                handle_in_pass3(bldg);
            }
            return false;
        }

        // Remove all buildings that do not overlap at all.
        std::erase_if(cand_bldgs, [&](const auto* bldg) {
            auto* bldg_comp = get_comp_ptr(bldg);
            return 
                bldg_comp->has_ht_top && 
                !bldg_comp->parts_only &&
                (part_comp->ht_top < bldg_comp->ht_btm ||
                    part_comp->ht_btm > bldg_comp->ht_top);
        });
        if (cand_bldgs.size() == 1) {
            add_mapping(cand_bldgs[0]);
            return true;
        }

        // if part fits perfectly inside a _single_ building, done!
        auto res = get_one_of(cand_bldgs.begin(), cand_bldgs.end(), [&](const auto* bldg)
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            return bldg_comp->has_ht_top &&
                (part_comp->ht_btm >= bldg_comp->ht_btm &&
                    part_comp->ht_top <= bldg_comp->ht_top);
        });
        if (res != cand_bldgs.end()) {
            add_mapping(*res);
            return true;
        }

        // For pass 3, only consider buildings that lack height info,
        // must contain parts, or overlap the part in height by at least 50%
        double part_height = part_comp->ht_top - part_comp->ht_btm;
        for (const auto* bldg : cand_bldgs)
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            if (bldg_comp->parts_only || 
                !bldg_comp->has_ht_top ||
                part_comp->overlap(*bldg_comp) >= 0.5 * part_height)
            {
                handle_in_pass3(bldg);
            }
        }
        return false;
    }

    bool map_part_pass3(const bldg_comp* part_comp, 
        const std::vector<const bldg_comp*>& cand_bldg_comps, part_map_log& log)
    {
        // todo: assign it to the one with maximum volume intersection?
        // (only if both part and building have heights and were not defaulted),
        // with a minimum coverage of 75% (unless the building is parts only).
        return false;
    }

private:
    const osm_mesh_object_db* m_obj_db;
    const std::vector<bldg_comp>& m_all_comps;
    const std::vector<const bldg_comp*>& m_part_comps;

    comp_mapping_t m_pass3_part2bldg;
    comp_mapping_t m_pass3_bldg2part;

    std::vector<const bldg_comp*> m_unmapped_parts;
    comp_mapping_t m_bldg2part_map;
};


static inline void mesh_add_triangle(draw_datad& mesh, glm::u32vec3 indices, bool reverse_winding)
{
    if (reverse_winding) {
        mesh.add_triangle(indices[0], indices[2], indices[1]);
    } else {
        mesh.add_triangle(indices[0], indices[1], indices[2]);
    }
}

static uint32_t mesh_add_polygon(draw_datad& mesh, const auto& nodes,
    std::span<const uint32_t> indices, double height, bool reverse_winding = false)
{
    uint32_t vert_startidx = uint32_t(mesh.num_verts());
    for (const auto& node : nodes) {
        mesh.add_vertex(vert_x(node), vert_y(node), height);
    }

    for (size_t i = 0; i < indices.size(); i += 3)
    {
        uint32_t idx0 = indices[i] + vert_startidx;
        uint32_t idx1 = indices[i + 1] + vert_startidx;
        uint32_t idx2 = indices[i + 2] + vert_startidx;

        mesh_add_triangle(mesh, { idx0, idx1, idx2 }, reverse_winding);
    }
    return vert_startidx;
}

// \param verts_ccw True if the vertices are in CCW order.
static void mesh_add_sides(draw_datad& mesh, uint32_t bot_verts_idx,
    uint32_t top_verts_idx, uint32_t num_verts, bool verts_ccw = true)
{
    for (uint32_t icur = 0; icur < num_verts; ++icur)
    {
        uint32_t inext = (icur + 1) % num_verts;
        uint32_t quad[4] = {
            bot_verts_idx + icur,
            bot_verts_idx + inext,
            top_verts_idx + icur,
            top_verts_idx + inext,
        };
        mesh_add_triangle(mesh, { quad[0], quad[2], quad[3] }, verts_ccw);
        mesh_add_triangle(mesh, { quad[0], quad[3], quad[1] }, verts_ccw);
    }
}

static bool generate_comp_mesh(draw_datad& mesh, const bldg_comp& comp, const osm_mesh_object_db* obj_db)
{
    auto& obj = obj_db->get<osm_mesh_object>(comp.mesh_obj_idx);
    auto& obj_osm = obj_db->get<osm_area>(obj.osm_obj_idx);

    for (auto& poly : obj_osm.polys) 
    {
        auto tri_indices = polygon_triangulate(poly);
        auto poly_nodes = std::span(&poly.front().front(), poly.back().data() + poly.back().size()); // yuck

        assert(check_tris_oriented(poly_nodes, tri_indices, ORIENT_CCW));

        uint32_t bot_verts_idx = mesh_add_polygon(mesh, poly_nodes, tri_indices, comp.ht_btm);
        uint32_t top_verts_idx = mesh_add_polygon(mesh, poly_nodes, tri_indices, comp.ht_top, true);

        uint32_t vert_offset = 0;
        
        auto& outer_ring = poly[0];
        auto outer_size = uint32_t(outer_ring.size());
        mesh_add_sides(mesh, bot_verts_idx, top_verts_idx, outer_size);
        vert_offset += outer_size;

        for (size_t ihole = 1; ihole < poly.size(); ++ihole)
        {
            auto& hole = poly[ihole];
            auto hole_size = uint32_t(hole.size());
            auto inner_bot_idx = bot_verts_idx + vert_offset;
            auto inner_top_idx = top_verts_idx + vert_offset;

            mesh_add_sides(mesh, inner_bot_idx, inner_top_idx, hole_size, false);
            vert_offset += hole_size;
        }
    }
    return true;
}

bool bldg_comp_builder::do_build_all(const osm_mesh_object_db* obj_db, 
    const std::vector<bldg_comp>& all_comps, std::vector<draw_datad>& out_drawdata)
{
    auto tbegin = clk::now();

    std::vector<const bldg_comp*> bldg_comps, part_comps;
    timeit("Bucketing buildings/parts", [&]()
    {    
        for (const auto& comp : all_comps) {
            if (comp.is_part) {
                part_comps.push_back(&comp);
            } else {
                bldg_comps.push_back(&comp);
            }
        }
    });
    logMESSAGE("%zu buildings, %zu parts", bldg_comps.size(), part_comps.size());

    bldg_part_mapper part_mapper(obj_db, all_comps, part_comps);
    timeit("Mapping parts to buildings", [&]() {
        part_mapper.do_mapping();

        if (!part_mapper.unmapped_parts().empty()) {
            logWARNING("%zu/%zu part(s) could not be mapped to a parent building",
                part_mapper.unmapped_parts().size(), part_comps.size());
        }
    });

    const glm::vec4 building_color(0.85f, 0.75f, 0.65f, 1.0f);

    size_t num_verts = 0, num_tris = 0;
    auto add_drawdata = [&]<class String>(draw_datad&& dd, String&& name) 
    {
        num_verts += dd.num_verts();
        num_tris += dd.num_tris();

        dd.color = building_color;
        dd.name = std::forward<String>(name);
        out_drawdata.push_back(std::move(dd));
    };

    auto comp_osm_id = [&](const bldg_comp* comp) {
        auto& obj = obj_db->get<osm_mesh_object>(comp->mesh_obj_idx);
        return obj_db->obj_osm_id(obj);
    };

    // Build meshes from the parts
    timeit("Building meshes", [&]()
    {
        auto& unmapped_parts = part_mapper.unmapped_parts();
        auto& bldg2parts = part_mapper.bldg2parts();

        for (auto* bldg_comp : bldg_comps)
        {
            auto& bldg = obj_db->get<osm_mesh_object>(bldg_comp->mesh_obj_idx);
            auto& bldg_osm = obj_db->get<osm_area>(bldg.osm_obj_idx);
            auto bldg_osm_id = obj_db->obj_osm_id(bldg);

            if (bldg.name.empty()) {
                logDEBUG(LOG_MESSAGE, "Adding building %lld", bldg_osm_id);
            } else {
                logDEBUG(LOG_MESSAGE, "Adding building %lld (%s)", bldg_osm_id, bldg.name.c_str());
            }
            
            auto bldg_parts_it = bldg2parts.find(bldg_comp);
            if (bldg_parts_it == bldg2parts.end())
            {
                draw_datad dd;
                if (generate_comp_mesh(dd, *bldg_comp, obj_db)) {
                    auto name = bldg.name.empty() ? "bldg " + std::to_string(bldg_osm_id) : bldg.name;
                    add_drawdata(std::move(dd), std::move(name));
                }
            } 
            else {
                for (auto* part_comp : bldg_parts_it->second)
                {
                    auto part_id = comp_osm_id(part_comp);
                    draw_datad dd;
                    if (generate_comp_mesh(dd, *part_comp, obj_db)) {
                        logDEBUG(LOG_MESSAGE, "   Adding part %lld", part_id);
                        add_drawdata(std::move(dd), "part " + std::to_string(part_id));
                    }
                }
            } 
        }

        for (auto* part_comp : unmapped_parts)
        {
            auto part_id = comp_osm_id(part_comp);
            draw_datad dd;
            if (generate_comp_mesh(dd, *part_comp, obj_db)) {
                add_drawdata(std::move(dd), "part " + std::to_string(part_id));
            }
        }
    });

    auto tend = clk::now();
    logMESSAGE("Generated %u tris and %u vertices in %s",
        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());

    return true;
}

#endif