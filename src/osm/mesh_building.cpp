
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include "containers/aabb_tree.hpp"
#include "geom.hpp"
#include "mesh.hpp"


namespace CGALPMP = CGAL::Polygon_mesh_processing;

template <typename Kernel>
using cgalmesh = CGAL::Surface_mesh<typename Kernel::Point_3>;

template <typename Kernel>
using cgalpoint = typename Kernel::Point_3;


bool mesh_builder::get_building_part(const building_info& info, building_part& out_part)
{
    auto& nodes = info.way.nodes;
    if (nodes.empty() || !nodes.is_closed()) {
        return false;
    }

    bbox2d bbox;
    std::vector<glm::dvec2> verts;
    verts.resize(nodes.size() - 1);

    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        auto proj = osmium::geom::MercatorProjection{}(nodes[i].location());
        auto proj_glm = glm::dvec2(proj.x, proj.y);
        bbox.extend(proj_glm);
        verts[i] = proj_glm;
    }

    out_part = {
        .id = info.way.id,
        .orient = polygon_orient(verts),
        .bbox = bbox,
        .ht_btm = info.ht_btm,
        .ht_top = info.ht_top,
        .verts = std::move(verts)
    };
    return true;
}

bool mesh_builder::add_building(const building_info& info)
{
    building_part part;
    if (!get_building_part(info, part)) {
        logERROR("Failed to get part for way %lld", info.way.id);
        return false;
    }

    if (info.is_part) {
        m_building_parts.push_back(std::move(part));
    }
    else {
        m_buildings.push_back({
            .info = std::move(part),
            .name = info.way.name ? info.way.name : "",
            .parts = {},
        });
    }
    return true;
}

//void add_line_indices(uint32_t idx0, uint32_t idx1)
//{
//    m_line_indices.push_back(idx0);
//    m_line_indices.push_back(idx1);
//    // prim restart index
//    m_line_indices.push_back(std::numeric_limits<uint32_t>::max());
//}
//template <typename TMesh>
//void add_polyline_indices(uint32_t startidx, uint32_t size, bool is_closed)
//{
//    if (size == 0) { return; } // handle underflow
//
//    for (uint32_t i = 0; i < size - 1; ++i) {
//        m_line_indices.push_back(startidx + i);
//    }
//    if (is_closed) {
//        m_line_indices.push_back(startidx);
//    } else {
//        m_line_indices.push_back(startidx + size - 1);
//    }
//    // prim restart index
//    m_line_indices.push_back(std::numeric_limits<uint32_t>::max());
//}
//
//void add_polyline_from_nodes(const osmium::NodeRefList& nodes)
//{
//    uint32_t vert_startidx = uint32_t(m_verts.size() / 3);
//
//    for (const auto& node : nodes) {
//        auto loc = osmium::geom::MercatorProjection{}(node.location());
//        m_verts.push_back(loc.x);
//        m_verts.push_back(loc.y);
//        m_verts.push_back(0.0);
//    }
//    add_polyline_indices(vert_startidx, uint32_t(nodes.size()), nodes.is_closed());
//}
//
//void add_polyline(const std::vector<coord>& verts, double height, bool is_closed)
//{
//    uint32_t vert_startidx = uint32_t(m_verts.size() / 3);
//
//    for (const auto& vert : verts) {
//        m_verts.push_back(vert.x);
//        m_verts.push_back(vert.y);
//        m_verts.push_back(height);
//    }
//
//    add_polyline_indices(vert_startidx, uint32_t(verts.size()), is_closed);
//}
//double cross(coord v, coord w) {
//    return v.x * w.y - v.y * w.x; 
//}
//
//double orient(coord a, coord b, coord c) { 
//    return cross(coord{ b.x - a.x, b.y - a.y }, coord{ c.x - a.x, c.y - a.y });
//}
//
//bool isConvex(vector<pt> p) {
//    bool hasPos = false, hasNeg = false;
//    for (int i = 0, n = p.size(); i < n; i++) {
//        int o = orient(p[i], p[(i + 1) % n], p[(i + 2) % n]);
//        if (o > 0) hasPos = true;
//        if (o < 0) hasNeg = true;
//    }
//    return !(hasPos && hasNeg);
//}

template <typename Kernel>
struct cgalmesh_builder
{
    cgalmesh_builder(cgalmesh<Kernel>& m) :
        m_mesh(m)
    {}

    void add_vertex(double x, double y, double z) {
        m_vmap.push_back(m_mesh.add_vertex(cgalpoint<Kernel>(x, y, z)));
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        m_mesh.add_face(m_vmap[idx0], m_vmap[idx1], m_vmap[idx2]);
    }

    size_t num_verts() const { return m_vmap.size(); }

private:
    cgalmesh<Kernel>& m_mesh;
    std::vector<typename cgalmesh<Kernel>::Vertex_index> m_vmap;
};

static inline void mesh_add_triangle(auto& mesh, glm::u32vec3 indices, bool reverse_winding)
{
    if (reverse_winding) {
        mesh.add_triangle(indices[0], indices[2], indices[1]);
    } else {
        mesh.add_triangle(indices[0], indices[1], indices[2]);
    }
}

static uint32_t mesh_add_polygon(auto& mesh,
    std::span<const glm::dvec2> verts,
    std::span<const uint32_t> indices,
    double height,
    bool reverse_winding = false)
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

static void gen_building_part_mesh(auto& mesh, const mesh_builder::building_part& part)
{
    assert(part.orient != ORIENT_COLL);

    auto tri_indices = polygon_triangulate(part.verts, part.orient);

    uint32_t bot_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_btm, true);
    uint32_t top_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_top);

    // sides
    for (uint32_t icur = 0; icur < part.verts.size(); ++icur)
    {
        uint32_t inext = (icur + 1) % part.verts.size();

        uint32_t quad[4] = {
            bot_verts_idx + icur,
            bot_verts_idx + inext,
            top_verts_idx + icur,
            top_verts_idx + inext,
        };
        // faces
        //mesh_add_triangle(mesh, quad[0], quad[3], quad[2]);
        //mesh_add_triangle(mesh, quad[0], quad[1], quad[3]);

        mesh_add_triangle(mesh, { quad[0], quad[2], quad[3] }, part.orient == ORIENT_CCW);
        mesh_add_triangle(mesh, { quad[0], quad[3], quad[1] }, part.orient == ORIENT_CCW);

        //double o = orient(part.coords[quad[0]],
        //    part.coords[quad[2]],
        //    part.coords[quad[3]]);
        //std::cout << o << "\n";

        // outlines
        //mesh.add_segment(quad[0], quad[2]);
        //mesh.add_segment(quad[1], quad[3]);
    }
}

static glm::vec4 building_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);

template <typename Kernel>
static draw_datad cgalmesh_draw_data(const cgalmesh<Kernel>& mesh, const std::string& name)
{
    draw_datad ret;
    ret.color = building_color;
    ret.name = name;
    ret.verts.reserve(mesh.num_vertices() * 3);
    ret.tri_indices.reserve(mesh.num_faces() * 3);

    for (const auto& v : mesh.vertices())
    {
        auto p = mesh.point(v);
        ret.verts.push_back(CGAL::to_double(p.x()));
        ret.verts.push_back(CGAL::to_double(p.y()));
        ret.verts.push_back(CGAL::to_double(p.z()));
    }

    for (const auto& f : mesh.faces())
    {
        auto h = mesh.halfedge(f);
        do {
            ret.tri_indices.push_back(mesh.target(h).id());
            h = mesh.next(h);
        } while (h != mesh.halfedge(f));
    }

    return ret;
}

// Check all the common failures for a CGAL mesh.
template <typename Kernel>
static bool cgalmesh_is_watertight(const cgalmesh<Kernel>& mesh, osmium::object_id_type id, const char* name)
{
    using halfedge_index = cgalmesh<Kernel>::halfedge_index;

    std::string idstr = "id " + std::to_string(id) + " (" + name + ")";

    if (!CGAL::is_triangle_mesh(mesh)) {
        logERROR("is_triangle_mesh() failed for mesh %s", idstr.c_str());
        return false;
    }

    if (!CGAL::is_valid_polygon_mesh(mesh)) {
        logERROR("is_valid_polygon_mesh() failed for mesh %s", idstr.c_str());
        return false;
    }

    if (!CGAL::is_closed(mesh))
    {
        logERROR("is_closed() failed for mesh %s", idstr.c_str());
        logMESSAGE("Debug info:");

        std::vector<halfedge_index> border_edges;
        CGALPMP::extract_boundary_cycles(mesh, std::back_inserter(border_edges));
        logMESSAGE("Mesh has %zu boundary cycles", border_edges.size());

        for (size_t i = 0; i < border_edges.size(); ++i)
        {
            logMESSAGE("Cycle %zu", i);

            halfedge_index start = border_edges[i];
            halfedge_index h = start;
            do {
                auto p1 = mesh.point(mesh.source(h));
                auto p2 = mesh.point(mesh.target(h));
                h = mesh.next(h);

                logMESSAGE("\tEdge from (%.8lf, %.8lf) to (%.8lf, %.8lf)",
                    p1.x(), p1.y(), p2.x(), p2.y());
            } while (h != start);
        }
        return false;
    }

    if (!CGALPMP::is_outward_oriented(mesh)) {
        logERROR("is_outward_oriented() failed for mesh %s", idstr.c_str());
        return false;
    }

    if (!CGALPMP::does_bound_a_volume(mesh)) {
        logERROR("does_bound_a_volume() failed for mesh %s", idstr.c_str());
        return false;
    }

    return true;
}

bool mesh_builder::gen_building_drawdata(std::vector<draw_datad>& drawdata, aabb_tree<building*>* bldg_tree_ptr)
{
    auto& bldg_tree = *bldg_tree_ptr;

    auto tree_ptrs = std::make_unique_for_overwrite<building*[]>(m_buildings.size());
    for (size_t i = 0; i < m_buildings.size(); ++i) {
        tree_ptrs[i] = &m_buildings[i];
    }
    bldg_tree = aabb_tree<building*>::create_unsafe({ tree_ptrs.get(), m_buildings.size() });

    std::vector<building_part*> unmapped_parts;
    for (auto& part : m_building_parts)
    {
        if (part.orient == ORIENT_COLL) {
            logWARNING("Ignoring part %lld as it has 0 area", part.id);
            continue;
        }

        building* mapped_bldg = nullptr;
        auto inter_bldgs = bldg_tree.intersect(part.bbox);
        if (inter_bldgs.empty())
        {
            logWARNING("Part %lld does not intersect any buildings", part.id);
            unmapped_parts.push_back(&part);
            continue;
        }
        else {
            for (size_t icand = 0; icand < inter_bldgs.size() - 1; ++icand) {
                if (polygon_covered_by(part.verts, inter_bldgs[icand]->info.verts)) {
                    mapped_bldg = inter_bldgs[icand];
                    break;
                }
            }
            // Last, skip polygon containment check
            if (!mapped_bldg) { mapped_bldg = inter_bldgs.back(); }
        }

        if (mapped_bldg) {
            mapped_bldg->parts.push_back(&part);
        } else {
            unmapped_parts.push_back(&part);
        }
    }

    if (!unmapped_parts.empty()) {
        logWARNING("%zu building part(s) could not be "
            "mapped to a building", unmapped_parts.size());
    }

    // build meshes from the parts and union the parts
    for (auto& building : m_buildings)
    {
        draw_datad dd;
        dd.color = building_color;

        if (building.name.empty()) {
            logMESSAGE("Adding building %lld", building.info.id);
        } else {
            logMESSAGE("Adding building %lld (%s)", building.info.id, building.name.c_str());
        }

        if (building.parts.empty())
        {
            dd.name = building.name;
            gen_building_part_mesh(dd, building.info);
        }
        else {
            // inexact kernel crashes with some meshes
            using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

            cgalmesh<Kernel> mesh;
            cgalmesh_builder<Kernel> mesh_builder(mesh);
            gen_building_part_mesh(mesh_builder, building.info);

            assert(cgalmesh_is_watertight<Kernel>(mesh, building.info.id, building.name.c_str()));

            for (size_t i = 0; i < building.parts.size(); ++i)
            {
                building_part& part = *building.parts[i];
                logMESSAGE("   Joining part %d", part.id);

                cgalmesh<Kernel> part_mesh;
                cgalmesh_builder<Kernel> part_mesh_builder(part_mesh);
                gen_building_part_mesh(part_mesh_builder, part);

                assert(cgalmesh_is_watertight<Kernel>(part_mesh, part.id, building.name.c_str()));

                cgalmesh<Kernel> result_mesh;
                if (!CGALPMP::corefine_and_compute_union(mesh, part_mesh, result_mesh))
                {
                    logWARNING("Failed to join part %lld to building %lld (%s)",
                        part.id, building.info.id, building.name.c_str());
                    unmapped_parts.push_back(&part);
                    continue;
                }
                mesh = std::move(result_mesh);
            }

            CGALPMP::remove_degenerate_faces(mesh);
            CGALPMP::remove_isolated_vertices(mesh);
            //CGALPMP::stitch_borders(mesh);
            //CGALPMP::merge_duplicate_vertices(mesh);

            dd = cgalmesh_draw_data<Kernel>(mesh, building.name);
        }

        drawdata.push_back(std::move(dd));
    }

    for (auto* part : unmapped_parts)
    {
        draw_datad dd;
        dd.color = building_color;
        dd.name = "part " + std::to_string(part->id);
        gen_building_part_mesh(dd, *part);

        drawdata.push_back(std::move(dd));
    }

    return true;
}