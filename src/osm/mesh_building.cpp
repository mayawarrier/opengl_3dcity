#include <osmium/osm/node_ref_list.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/area.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/container/flat_set.hpp>

#include "containers/aabb_tree.hpp"
#include "geom.hpp"
#include "mesh.hpp"


static const osmium::OSMObject* bldg_object(const mesh_builder::building_info& info)
{
    switch (info.obj_type) {
        case OBJ_TYPE_WAY:  return (const osmium::OSMObject*)info.way;
        case OBJ_TYPE_AREA: return (const osmium::OSMObject*)info.area;
        default: assert(false); return nullptr;
    }
}

static const osmium::object_id_type bldg_id(const mesh_builder::building_info& info)
{
    switch (info.obj_type) {
        case OBJ_TYPE_WAY:  return info.way->id();
        case OBJ_TYPE_AREA: return info.area->orig_id();
        default: assert(false); return -1;
    }
}

static void set_building_heights(const osmium::TagList& tags, mesh_builder::building_part& part)
{
    int levels, min_level;
    double height, min_height;

    bool has_levels =    parse_num_if_exists(tags["building:levels"], levels);
    bool has_minlevel =  parse_num_if_exists(tags["building:min_level"], min_level);
    bool has_height =    parse_num_if_exists(tags["height"], height);
    bool has_minheight = parse_num_if_exists(tags["min_height"], min_height);

    part.ht_top = (has_height && height > 0) ? height : 
                  (has_levels && levels > 0) ? (3.0 * levels) : 
                  -1.0;
    part.ht_btm = (has_minheight && min_height >= 0) ? min_height : 
                  (has_minlevel && min_level >= 0) ? (3.0 * min_level) :
                  -1.0;

    bool valid_range = part.ht_top > part.ht_btm;
    part.has_ht_btm = part.ht_btm >= 0.0 && valid_range;
    part.has_ht_top = part.ht_top > 0.0 && valid_range;

    if (!part.has_ht_btm) { part.ht_btm = 0.0; }
    if (!part.has_ht_top) { part.ht_top = part.ht_btm + 3.0; }
}

bool mesh_builder::get_building_part(const building_info& info, building_part& out_part)
{
    auto project_and_extend_bbox = [](const osmium::Location & loc, bbox2d & bbox) -> glm::dvec2
    {
        auto proj = osmium::geom::MercatorProjection{}(loc);
        auto proj_glm = glm::dvec2(proj.x, proj.y);
        bbox.extend(proj_glm);
        return proj_glm;
    };

    switch (info.obj_type) 
    {
        case OBJ_TYPE_WAY:
        {
            auto& nodes = info.way->nodes();
            if (nodes.empty() || !nodes.is_closed()) {
                return false;
            }

            auto bbox = bbox2d::empty();
            std::vector<glm::dvec2> verts(nodes.size() - 1);
            for (size_t i = 0; i < nodes.size() - 1; ++i) {
                verts[i] = project_and_extend_bbox(nodes[i].location(), bbox);
            }
            if (path_orient(verts) == ORIENT_CW) {
                std::reverse(verts.begin(), verts.end());
            }

            out_part = {
                .id = info.way->id(),
                .verts = std::move(verts),
                .bbox = bbox,
                .obj_type = OBJ_TYPE_WAY
            };
            set_building_heights(bldg_object(info)->tags(), out_part);
        }
        break;

        case OBJ_TYPE_AREA:
        {
            auto& area = *info.area;
            if (area.outer_rings().empty()) {
                return false;
            }
            
            auto bbox = bbox2d::empty();
            std::vector<glm::dvec2> verts;
            std::vector<std::pair<int, int>> ring_sizes;

            for (auto& outer_ring : area.outer_rings())
            {
                if (!outer_ring.is_closed() || outer_ring.size() < 3) {
                    return false;
                }
                for (size_t i = 0; i < outer_ring.size() - 1; ++i) {
                    auto& vert = outer_ring[i];
                    verts.push_back(project_and_extend_bbox(vert.location(), bbox));
                }
                ring_sizes.push_back({ 0, int(outer_ring.size()) - 1 });
            
                for (auto& inner_ring : area.inner_rings(outer_ring))
                {
                    if (!inner_ring.is_closed() || inner_ring.size() < 3) {
                        return false;
                    }
                    int startidx = int(verts.size());
                    for (size_t i = 0; i < inner_ring.size() - 1; ++i) {
                        auto& vert = inner_ring[i];
                        verts.push_back(project_and_extend_bbox(vert.location(), bbox));
                    }
                    ring_sizes.push_back({ startidx, int(inner_ring.size()) - 1 });
                }
            }

            decltype(area::rings) rings(ring_sizes.size());
            for (size_t i = 0; i < ring_sizes.size(); ++i) 
            {
                auto [startidx, size] = ring_sizes[i];        
                auto ring = std::span(&verts[startidx], size);

                // outer ring must be CCW, inner rings CW
                if (i == 0) {
                    if (path_orient(ring) == ORIENT_CW) {
                        std::reverse(ring.begin(), ring.end());
                    }
                }
                else if (path_orient(ring) == ORIENT_CCW) {
                    std::reverse(ring.begin(), ring.end());
                }
                rings[i] = ring;
            }

            m_bldg_areas[area.orig_id()] = { .rings = std::move(rings) };
            
            out_part = {
                .id = area.orig_id(),
                .verts = std::move(verts),
                .bbox = bbox,
                .obj_type = OBJ_TYPE_AREA
            };
            set_building_heights(bldg_object(info)->tags(), out_part);
        }
        break;

        default: return false;
    }
    
    return true;
}

bool mesh_builder::add_building(const building_info& info)
{
    building_part part;
    if (!get_building_part(info, part)) {
        logWARNING("Failed to get part for way %lld", bldg_id(info));
        return false;
    }

    if (info.is_part) {
        m_building_parts.push_back(std::move(part));
    }
    else {
        const char* name = bldg_object(info)->tags()["name"];
        m_buildings.push_back({
            .base = std::move(part),
            .name = name ? name : "",
            .parts = {},
        });
    }
    return true;
}

static inline void mesh_add_triangle(draw_datad& mesh, glm::u32vec3 indices, bool reverse_winding)
{
    if (reverse_winding) {
        mesh.add_triangle(indices[0], indices[2], indices[1]);
    } else {
        mesh.add_triangle(indices[0], indices[1], indices[2]);
    }
}

static uint32_t mesh_add_polygon(draw_datad& mesh, std::span<const glm::dvec2> verts,
    std::span<const uint32_t> indices, double height, bool reverse_winding = false)
{
    uint32_t vert_startidx = uint32_t(mesh.num_verts());
    for (const auto& vert : verts) {
        mesh.add_vertex(vert.x, vert.y, height);
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

bool mesh_builder::get_building_part_mesh(draw_datad& mesh, const building_part& part)
{
    std::vector<uint32_t> tri_indices;
    if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
        tri_indices = polygon_triangulate(m_bldg_areas[part.id].rings);
    } else {
        auto vert_span = std::span<const glm::dvec2>(part.verts);
        tri_indices = polygon_triangulate(std::span(&vert_span, 1));
    }

    assert(check_triangles_oriented(part.verts, tri_indices));

    uint32_t bot_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_btm);
    uint32_t top_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_top, true);

    if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] 
    {
        auto& rings = m_bldg_areas[part.id].rings;

        auto& outer_ring = rings[0];
        mesh_add_sides(mesh, bot_verts_idx, top_verts_idx, uint32_t(outer_ring.size()));

        uint32_t vert_offset = uint32_t(outer_ring.size());
        for (size_t iring = 1; iring < rings.size(); ++iring) 
        {
            auto& inner_ring = rings[iring];
            uint32_t inner_ring_size = uint32_t(inner_ring.size());
            uint32_t inner_bot_idx = bot_verts_idx + vert_offset;
            uint32_t inner_top_idx = top_verts_idx + vert_offset;

            mesh_add_sides(mesh, inner_bot_idx, inner_top_idx, inner_ring_size, false);
            vert_offset += inner_ring_size;
        }
    }
    else {
        mesh_add_sides(mesh, bot_verts_idx, top_verts_idx, uint32_t(part.verts.size()));
    }
    return true;
}

bool mesh_builder::gen_building_drawdata(std::vector<draw_datad>& drawdata, aabb_tree2d<building*>* bldg_tree_ptr)
{
    int CUR_STEP = 1;
    constexpr int NUM_STEPS = 3;

    auto step_done = [&](const char* msg, clk::duration dur) {
        logMESSAGE("  [%d/%d] %s: %s", CUR_STEP, NUM_STEPS, msg, clock_dur_str(dur).c_str());
        CUR_STEP++;
    };

    size_t num_verts = 0, num_tris = 0;
    auto add_drawdata = [&](draw_datad&& dd) {
        num_verts += dd.num_verts();
        num_tris += dd.num_tris();
        drawdata.push_back(std::move(dd));
    };

    logMESSAGE("-----------------------------------------------");
    logMESSAGE("Generating buildings...");
    logMESSAGE("%zu buildings, %zu parts, %zu areas", 
        m_buildings.size(), m_building_parts.size(), m_bldg_areas.size());

    auto tbegin = clk::now();

    // Build AABB tree for fast intersection queries
    auto tbegin_tree = clk::now();
    auto& bldg_tree = *bldg_tree_ptr;
    {
        buffer<building*> tree_objects(m_buildings.size(), buffer_overwrite);
        for (size_t i = 0; i < m_buildings.size(); ++i) {
            tree_objects.ptr[i] = &m_buildings[i];
        }
        bldg_tree = aabb_tree2d<building*>::create_unsafe(tree_objects.span());
    }
    step_done("Building AABB tree", clk::now() - tbegin_tree);

    // Map parts/areas to buildings
    auto tbegin_map = clk::now();
    std::vector<building_part*> unmapped_parts;
    {
        for (auto& part : m_building_parts)
        {
            auto inter_bldgs = bldg_tree.query_bbox_all(part.bbox);
            if (inter_bldgs.empty()) {
                logDEBUG(LOG_WARNING, "Part %lld tree query returned nothing", part.id);
                unmapped_parts.push_back(&part);
                continue;
            }

            polygon_cspan part_poly;
            std::span<const glm::dvec2> part_verts;

            if (part.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
                part_poly = m_bldg_areas[part.id].rings;
            } else {
                part_verts = part.verts;
                part_poly = std::span(&part_verts, 1);
            }

            building* mapped_bldg = nullptr;
            for (size_t icand = 0; icand < inter_bldgs.size(); ++icand) 
            {
                polygon_cspan bldg_poly;
                std::span<const glm::dvec2> bldg_verts;

                if (inter_bldgs[icand]->base.obj_type == OBJ_TYPE_AREA) [[ unlikely ]] {
                    bldg_poly = m_bldg_areas[inter_bldgs[icand]->base.id].rings;
                } else {
                    bldg_verts = inter_bldgs[icand]->base.verts;
                    bldg_poly = std::span(&bldg_verts, 1);
                }
            
                if (polygon_covered_by(part_poly, bldg_poly)) {
                    mapped_bldg = inter_bldgs[icand];
                    break;
                }
            }
            if (mapped_bldg) {
                mapped_bldg->parts.push_back(&part);
            } else {
                logDEBUG(LOG_MESSAGE, "Could not map part %lld to a building", part.id);
                logDEBUG(LOG_MESSAGE, "Tried candidates:");
                for (auto& bldg : inter_bldgs) {
                    logDEBUG(LOG_MESSAGE, "  Building %lld", bldg->base.id);
                }
                unmapped_parts.push_back(&part);
            }
        }

        if (!unmapped_parts.empty()) {
            logWARNING("%zu/%zu part(s) could not be mapped to a building", 
                unmapped_parts.size(), m_building_parts.size());
        }
    }
    step_done("Mapping parts to buildings", clk::now() - tbegin_map);

    const glm::vec4 building_color(0.85f, 0.75f, 0.65f, 1.0f);

    // Build meshes from the parts
    auto tbegin_mesh = clk::now();
    {
        for (auto& building : m_buildings)
        {
            if (building.name.empty()) {
                logDEBUG(LOG_MESSAGE, "Adding building %lld", building.base.id);
            } else {
                logDEBUG(LOG_MESSAGE, "Adding building %lld (%s)", building.base.id, building.name.c_str());
            }

            if (building.parts.empty() || building.base.has_ht_top)
            {
                draw_datad dd;
                if (get_building_part_mesh(dd, building.base)) {
                    dd.color = building_color;
                    dd.name = building.name.empty() ? "bldg " + std::to_string(building.base.id) : building.name;
                    add_drawdata(std::move(dd));
                }
            }
            for (auto* part : building.parts)
            {
                logDEBUG(LOG_MESSAGE, "   Adding part %lld", part->id);

                draw_datad dd;
                if (get_building_part_mesh(dd, *part)) {
                    dd.color = building_color;
                    dd.name = "part " + std::to_string(part->id);
                    add_drawdata(std::move(dd));
                }
            }
        }

        for (auto* part : unmapped_parts)
        {
            draw_datad dd;
            if (get_building_part_mesh(dd, *part)) {
                dd.color = building_color;
                dd.name = "part " + std::to_string(part->id);
                add_drawdata(std::move(dd));
            }
        }
    }
    step_done("Building meshes", clk::now() - tbegin_mesh);

    auto tend = clk::now();
    logMESSAGE("Generated %u tris and %u vertices in %s",
        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());

    return true;
}