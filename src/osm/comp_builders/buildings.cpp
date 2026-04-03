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


void bldg_comp_builder::set_bldg_heights(const osmium::TagList& tags, building_comp& comp)
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
    class logger
    {
    public:
        enum log_stage_type
        {
            STAGE_INTER,
            STAGE_COVER,
            STAGE_PASS2,
            STAGE_PASS3,
            NUM_STAGES
        };

        struct log
        {
            int stage;
            osmium::object_id_type part_id;
            std::vector<osmium::object_id_type> inter_bldg_ids;
            std::vector<osmium::object_id_type> covered_bldg_ids;
            std::vector<osmium::object_id_type> pass2_bldg_ids;
            std::vector<osmium::object_id_type> pass3_bldg_ids;
        };

    public:
        logger(const mesh_entity_db* obj_db) :
            m_entity_db(obj_db)
        {}

        log& new_log(const building_comp* part_comp, osmium::object_id_type id)
        {
            auto& log = m_logs[part_comp];
            log.part_id = id;
            log.stage = -1;
            return log;
        }

        log& get_log(const building_comp* part_comp) {
            return m_logs.at(part_comp);
        }

        void log_stage(log& log, log_stage_type stage, std::span<const mesh_entity*> bldgs)
        {
            auto& log_vec = [&]() -> std::vector<osmium::object_id_type>& {
                switch (stage) {
                    case STAGE_INTER: return log.inter_bldg_ids;
                    case STAGE_COVER: return log.covered_bldg_ids;
                    case STAGE_PASS2: return log.pass2_bldg_ids;
                    case STAGE_PASS3: return log.pass3_bldg_ids;
                }
                assert_msg(false, "Invalid stage");
                return log.inter_bldg_ids; // silence compiler
            }();

            for (const auto* bldg : bldgs) {
                log_vec.push_back(m_entity_db->ent_osm_id(*bldg));
            }     
            log.stage = stage;
        }

        void log_unmapped_parts(std::span<const building_comp*> unmapped_parts, const bbox2d& bldgs_bbox) const
        {
            std::vector<const building_comp*> boundary_uparts;
            std::array<std::vector<const building_comp*>, NUM_STAGES> uparts_per_stage;
        
            bbox2d scaled_bbox = bldgs_bbox.scaled(0.8);
            for (const auto* part_comp : unmapped_parts) 
            {
                auto& log = m_logs.at(part_comp);
                if (log.stage >= 0) 
                {
                    auto& part = m_entity_db->get<mesh_entity>(part_comp->entity_idx);
                    if (part.bbox.inside(scaled_bbox)) {
                        uparts_per_stage[log.stage].push_back(part_comp);
                    } else {
                        boundary_uparts.push_back(part_comp);
                    }
                }
            }
            for (int stage = 0; stage < NUM_STAGES; ++stage) 
            {
                if (!uparts_per_stage[stage].empty()) {
                    logWARNING("Unmapped parts after %s", stage_name(log_stage_type(stage)));
                    for (const auto* part_comp : uparts_per_stage[stage]) {
                        log_failure(m_logs.at(part_comp));
                    }
                }
            }
            // Logged separately since these are likely unmapped only because of missing data
            if (!boundary_uparts.empty()) {
                logWARNING("Unmapped parts near or at map boundary");
                for (const auto* part_comp : boundary_uparts) {
                    log_failure(m_logs.at(part_comp));
                }
            }
        }

    private:
        static void log_failure(const log& log)
        {
            std::string logstr = "Part " + std::to_string(log.part_id) + ": ";

            if (log.inter_bldg_ids.empty()) {
                logstr += "No intersecting buildings";
            }
            else {
                logstr += "\n";
                logstr += "    Intersected: ";
                logstr += str_join(log.inter_bldg_ids, ", ");
                logstr += "\n";

                if (!log.covered_bldg_ids.empty()) {
                    logstr += "    Covered: ";
                    logstr += str_join(log.covered_bldg_ids, ", ");
                    logstr += "\n";
                }
                if (!log.pass2_bldg_ids.empty()) {
                    logstr += "    After pass 2: ";
                    logstr += str_join(log.pass2_bldg_ids, ", ");
                    logstr += "\n";
                }
                if (!log.pass3_bldg_ids.empty()) {
                    logstr += "    After pass 3: ";
                    logstr += str_join(log.pass3_bldg_ids, ", ");
                    logstr += "\n";
                }
            }

            if (logstr.back() == '\n') {
                logstr.pop_back();
            }
            logMESSAGE(logstr.c_str());
        }

        const char* stage_name(log_stage_type stage) const
        {
            switch (stage) {
                case STAGE_INTER: return "intersection";
                case STAGE_COVER: return "coverage check";
                case STAGE_PASS2: return "pass 2";
                case STAGE_PASS3: return "pass 3";
                default: return "unknown";
            }
        }

    private:
        const mesh_entity_db* m_entity_db;
        types::unord_flat_map<const building_comp*, log> m_logs;
    };

    using comp_mapping_t = types::unord_flat_map<const building_comp*, std::vector<const building_comp*>>;

public:
    using bldg2part_map_type = comp_mapping_t;

public:
    bldg_part_mapper(const mesh_entity_db* entity_db,
        const std::vector<building_comp>& all_comps,
        const std::vector<const building_comp*>& part_comps) :
        m_entity_db(entity_db),
        m_all_comps(all_comps),
        m_part_comps(part_comps),
        m_logger(entity_db)
    {}

    void do_mapping()
    {
        bbox2d bldgs_bbox;
        for (auto& comp : m_all_comps) {
            auto& bldg = m_entity_db->get<mesh_entity>(comp.entity_idx);
            bldgs_bbox.extend(bldg.bbox);
        }

        for (auto* part_comp : m_part_comps)
        {
            auto& part = m_entity_db->get<mesh_entity>(part_comp->entity_idx);
            auto& part_osm = m_entity_db->get<osm_area>(part.obj_idx);
            auto& log = m_logger.new_log(part_comp, part_osm.id);

            mesh_entity::comp_flags_t search_flags;
            search_flags.set(COMP_TYPE_BUILDING);

            // Get nearby buildings.
            auto cand_bldgs = m_entity_db->entity_tree.query_intersecting_bboxes(part.bbox, search_flags);

            m_logger.log_stage(log, m_logger.STAGE_INTER, cand_bldgs);
            if (cand_bldgs.empty()) {
                m_unmapped_parts.push_back(part_comp);
                continue;
            }

            // Remove buildings whose polygons do not fully contain the part.
            double part_area = multipoly_area(part_osm.polys);
            std::erase_if(cand_bldgs, [&](const auto* bldg) 
            {
                // larger tolerance for larger polygons
                static constexpr double AREA_TOL_FRAC = 1e-4;
                static constexpr double MIN_TOL = 0.1;

                auto& bldg_osm = m_entity_db->get<osm_area>(bldg->obj_idx);
                double bldg_area = multipoly_area(bldg_osm.polys);
                double area_tol = std::max(AREA_TOL_FRAC * (part_area + bldg_area), MIN_TOL);

                return !multipoly_covered_by(part_osm.polys, bldg_osm.polys, area_tol);
            });

            m_logger.log_stage(log, m_logger.STAGE_COVER, cand_bldgs);
            if (cand_bldgs.empty()) {
                m_unmapped_parts.push_back(part_comp);            
                continue;
            }

            if (cand_bldgs.size() == 1) {
                auto* bldg_comp = get_comp_ptr(cand_bldgs[0]);
                m_bldg2part_map[bldg_comp].push_back(part_comp);
            } else {
                map_part_pass2(part_comp, cand_bldgs, log);
            }
        }

        for (auto& [part_comp, bldg_comps] : m_pass3_part2bldg) {
            if (!map_part_pass3(part_comp, bldg_comps, m_logger.get_log(part_comp))) {
                m_unmapped_parts.push_back(part_comp);
            }
        }

        m_logger.log_unmapped_parts(m_unmapped_parts, bldgs_bbox);
    }

    const std::vector<const building_comp*>& unmapped_parts() const {
        return m_unmapped_parts;
    }

    const bldg2part_map_type& bldg2parts() const {
        return m_bldg2part_map;
    }

private:
    const building_comp* get_comp_ptr(const mesh_entity* ent) const {
        return &m_all_comps[m_entity_db->ent_comp_idx(*ent, COMP_TYPE_BUILDING)];
    }

    // Parts that reach here have multiple candidates that fully cover them (in 2D).
    bool map_part_pass2(const building_comp* part_comp,
         std::vector<const mesh_entity*>& cand_bldgs, logger::log& log)
    {
        auto handle_in_pass3 = [&](const mesh_entity* bldg) 
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            m_pass3_bldg2part[bldg_comp].push_back(part_comp);
            m_pass3_part2bldg[part_comp].push_back(bldg_comp);
            m_logger.log_stage(log, m_logger.STAGE_PASS2, std::span(&bldg, 1));
        };

        auto add_mapping = [&](const mesh_entity* bldg) 
        {
            auto* bldg_comp = get_comp_ptr(bldg);
            m_bldg2part_map[bldg_comp].push_back(part_comp);
            m_logger.log_stage(log, m_logger.STAGE_PASS2, std::span(&bldg, 1));
        };

        auto map_one_of = [&](auto pred) 
        {
            auto res = get_one_of(cand_bldgs, [&](const auto* bldg) {
                return std::invoke(pred, get_comp_ptr(bldg));
            });
            if (res != cand_bldgs.end()) {
                add_mapping(*res);
                return true;
            }
            return false;
        };

        if (part_comp->has_ht_top) 
        {
            // remove all buildings that do not overlap at all
            std::erase_if(cand_bldgs, [&](const auto* bldg) {
                auto* bldg_comp = get_comp_ptr(bldg);
                return
                    !bldg_comp->parts_only &&
                    bldg_comp->has_ht_top &&
                    (part_comp->ht_top < bldg_comp->ht_btm ||
                        part_comp->ht_btm > bldg_comp->ht_top);
                });
            if (cand_bldgs.size() == 1) {
                add_mapping(cand_bldgs[0]);
                return true;
            }

            // a _single_ building perfectly contains the part
            if (map_one_of([&](const building_comp* bldg_comp) {
                    return bldg_comp->has_ht_top &&
                        part_comp->ht_btm >= bldg_comp->ht_btm &&
                        part_comp->ht_top <= bldg_comp->ht_top;
                })) {
                return true;
            }
            // a _single_ building that must contain parts
            if (map_one_of([&](const building_comp* bldg_comp) {
                    return bldg_comp->parts_only;
                })) {
                return true;
            }
        }

        for (const auto* bldg : cand_bldgs) {
            handle_in_pass3(bldg);
        }
        return false;
    }

    // Parts that reach here lack heights, or have canddiates that
    // overlap the part in height, themselves lack heights, and/or must contain parts.
    bool map_part_pass3(const building_comp* part_comp, 
        const std::vector<const building_comp*>& cand_bldg_comps, logger::log& log)
    {
        // todo: assign it to the one with maximum coverage and height intersection?
        // (only if both part and building have heights and were not defaulted),
        // with a minimum coverage of 75% (unless the building is parts only).
        return false;
    }

private:
    const mesh_entity_db* m_entity_db;
    const std::vector<building_comp>& m_all_comps;
    const std::vector<const building_comp*>& m_part_comps;

    comp_mapping_t m_pass3_part2bldg;
    comp_mapping_t m_pass3_bldg2part;

    std::vector<const building_comp*> m_unmapped_parts;
    comp_mapping_t m_bldg2part_map;

    logger m_logger;
};


static inline glm::u32vec3 tri_indices(glm::u32vec3 indices, bool rev_winding) {
    return rev_winding ? glm::u32vec3(indices[0], indices[2], indices[1]) : indices;
}

static uint32_t dd_add_polygon(osm_tri_datad& dd, const auto& nodes,
    std::span<const uint32_t> indices, double height, bool rev_winding = false)
{
    uint32_t vert_startidx = dd.num_verts();
    for (const auto& node : nodes) {
        dd.add_vertex({ vert_x(node), vert_y(node), height });
    }
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        uint32_t vidx0 = indices[i] + vert_startidx;
        uint32_t vidx1 = indices[i + 1] + vert_startidx;
        uint32_t vidx2 = indices[i + 2] + vert_startidx;
        auto indices = tri_indices({ vidx0, vidx1, vidx2 }, rev_winding);
        dd.add_triangle(indices, TRI_TYPE_BUILDING);
    }
    return vert_startidx;
}

static void dd_add_ring_sides(osm_tri_datad& dd, uint32_t num_verts,
    uint32_t bot_verts_idx, uint32_t top_verts_idx, orient_t ring_orient)
{
    assert(ring_orient != ORIENT_COLL);

    bool rev_winding = (ring_orient != ORIENT_CCW);
    for (uint32_t icur = 0; icur < num_verts; ++icur)
    {
        uint32_t inext = (icur + 1) % num_verts;
        uint32_t quad[4] = {
            bot_verts_idx + icur,
            bot_verts_idx + inext,
            top_verts_idx + icur,
            top_verts_idx + inext,
        };
        auto indices1 = tri_indices({ quad[0], quad[3], quad[2] }, rev_winding);
        auto indices2 = tri_indices({ quad[0], quad[1], quad[3] }, rev_winding);
        dd.add_triangle(indices1, TRI_TYPE_BUILDING);
        dd.add_triangle(indices2, TRI_TYPE_BUILDING);
    }
}

static void generate_comp_mesh(osm_tri_datad& dd, const building_comp& comp, const mesh_entity_db* ent_db)
{
    auto& obj = ent_db->get<mesh_entity>(comp.entity_idx);
    auto& obj_osm = ent_db->get<osm_area>(obj.obj_idx);

    for (auto& poly : obj_osm.polys) 
    {
        auto tri_indices = polygon_triangulate(poly);
        auto poly_nodes = osm_area::poly_nodes(poly);

        assert(check_tris_winding(poly_nodes, tri_indices, ORIENT_CCW));

        uint32_t bot_verts_idx = dd_add_polygon(dd, poly_nodes, tri_indices, comp.ht_btm * ent_db->ht_scale, true);
        uint32_t top_verts_idx = dd_add_polygon(dd, poly_nodes, tri_indices, comp.ht_top * ent_db->ht_scale);
        
        auto& outer_ring = poly[0];
        uint32_t outer_size = uint32_t(outer_ring.size());
        dd_add_ring_sides(dd, outer_size, bot_verts_idx, top_verts_idx, ORIENT_CCW);
        
        uint32_t vert_offset = outer_size;
        for (size_t ihole = 1; ihole < poly.size(); ++ihole)
        {
            auto& hole = poly[ihole];
            uint32_t hole_size = uint32_t(hole.size());
            uint32_t hole_bot_idx = bot_verts_idx + vert_offset;
            uint32_t hole_top_idx = top_verts_idx + vert_offset;

            dd_add_ring_sides(dd, hole_size, hole_bot_idx, hole_top_idx, ORIENT_CW);
            vert_offset += hole_size;
        }
    }
}

bool bldg_comp_builder::do_build_all(const mesh_entity_db* entity_db, 
    const std::vector<building_comp>& all_comps, std::vector<osm_tri_datad>& out_tridata)
{
    auto tbegin = clk::now();

    auto timed_section = [&](const char* name, auto&& func) {
        log_func(name, std::forward<decltype(func)>(func), "Building mesher");
    };

    std::vector<const building_comp*> bldg_comps, part_comps;
    for (const auto& comp : all_comps) {
        if (comp.is_part) {
            part_comps.push_back(&comp);
        } else {
            bldg_comps.push_back(&comp);
        }
    }
    logMESSAGE("%zu buildings, %zu parts", bldg_comps.size(), part_comps.size());

    bldg_part_mapper part_mapper(entity_db, all_comps, part_comps);
    timed_section("Mapping parts to buildings", [&]() 
    {
        part_mapper.do_mapping();
        if (!part_mapper.unmapped_parts().empty()) {
            logWARNING("%zu/%zu part(s) could not be mapped to a parent building",
                part_mapper.unmapped_parts().size(), part_comps.size());
        }
    });

    osm_tri_datad dd;

    auto comp_osm_id = [&](const building_comp* comp) {
        auto& obj = entity_db->get<mesh_entity>(comp->entity_idx);
        return entity_db->ent_osm_id(obj);
    };

    timed_section("Generating meshes", [&]()
    {
        auto& unmapped_parts = part_mapper.unmapped_parts();
        auto& bldg2parts = part_mapper.bldg2parts();

        for (auto* bldg_comp : bldg_comps)
        {
            auto& bldg = entity_db->get<mesh_entity>(bldg_comp->entity_idx);
            auto& bldg_osm = entity_db->get<osm_area>(bldg.obj_idx);

            auto bldg_parts_it = bldg2parts.find(bldg_comp);
            if (bldg_parts_it == bldg2parts.end()) {
                generate_comp_mesh(dd, *bldg_comp, entity_db);
            } 
            else {
                // due to mapper error, parts often do not cover the entire parent
                // try to check if the coverage is high enough to skip drawing the parent
                bool draw_parts_only = true;
                if (bldg_comp->has_ht_top && !bldg_comp->parts_only)
                {
                    std::vector<const osm_area::multipoly_t*> part_polys;
                    for (auto* part_comp : bldg_parts_it->second)
                    {
                        auto& part = entity_db->get<mesh_entity>(part_comp->entity_idx);
                        auto& part_osm = entity_db->get<osm_area>(part.obj_idx);
                        part_polys.push_back(&part_osm.polys);
                    }
                    double parts_coverage = multipoly_coverage(std::span(part_polys), bldg_osm.polys);
                    draw_parts_only = parts_coverage > 0.8;
                }

                if (!draw_parts_only) {
                    generate_comp_mesh(dd, *bldg_comp, entity_db);
                }
                for (auto* part_comp : bldg_parts_it->second) {
                    generate_comp_mesh(dd, *part_comp, entity_db);
                }

                logMESSAGE("Parts for building %lld %s %s", 
                    bldg_osm.id,
                    bldg.name.empty() ? "" : ("(" + bldg.name + ")").c_str(),
                    draw_parts_only ? "" : "(with outline)");

                auto partids_str = str_join(bldg_parts_it->second, ", ", [&](const auto* comp, int index) 
                {
                    auto idstr = std::to_string(comp_osm_id(comp));
                    if (index != 0 && ((index % 8) == 0)) {
                        return "\n    " + idstr;
                    } else {
                        return idstr;
                    }
                });
                logMESSAGE("    %s", partids_str.c_str());
            } 
        }

        for (auto* part_comp : unmapped_parts) {
            generate_comp_mesh(dd, *part_comp, entity_db);
        }
    });

    auto tend = clk::now();
    logMESSAGE("Generated %u tris and %u vertices in %s",
        dd.num_tris(), dd.num_verts(), clock_dur_str(tend - tbegin).c_str());

    out_tridata.push_back(std::move(dd));
    return true;
}

#endif