
#include <algorithm>
#include <span>
#include <unordered_map>
#include <ranges>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/functional/hash.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include "../utils.hpp"
#include "mesh.hpp"


namespace ranges = std::ranges;

namespace CGALPMP = CGAL::Polygon_mesh_processing;

template <typename Kernel>
using cgalmesh = CGAL::Surface_mesh<typename Kernel::Point_3>;

template <typename Kernel>
using cgalpoint = typename Kernel::Point_3;


template <>
struct aabb_traits<mesh_builder::building*>
{
    static const bbox2d& get_bbox(mesh_builder::building* building) {
        return building->info.bbox;
    }
};

bool mesh_builder::get_building_part(const building_info& info, building_part& out_part)
{
    auto& nodes = info.way.nodes;
    if (nodes.empty() || !nodes.is_closed()) {
        return false;
    }

    bbox2d bbox;
    std::vector<osmpoint> verts;
    verts.resize(nodes.size() - 1);

    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        auto proj = osmium::geom::MercatorProjection{}(nodes[i].location());
        bbox.extend(glm::dvec2(proj.x, proj.y));
        verts[i] = proj;
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

bool mesh_builder::add_street(const street_info& info)
{
    std::vector<way_node> nodes;
    for (const auto& nr : info.way.nodes)
    {
        auto proj = osmium::geom::MercatorProjection{}(nr.location());
        nodes.push_back({ nr.ref(), proj });
    }

    m_streetways.push_back({
        .id = info.way.id,
        .name = info.way.name ? info.way.name : "",
        .width = info.width,
        .nodes = std::move(nodes)
    });

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

template <typename TPt>
struct drawdata_builder
{
    drawdata_builder(draw_data<TPt>& data) : 
        m_data(data) 
    {}

    void add_vertex(double x, double y, double z) {
        m_data.verts.push_back(x);
        m_data.verts.push_back(y);
        m_data.verts.push_back(z);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        m_data.tri_indices.push_back(idx0);
        m_data.tri_indices.push_back(idx1);
        m_data.tri_indices.push_back(idx2);
    }

    size_t num_verts() const { return m_data.verts.size() / 3; }

    void add_line(uint32_t idx0, uint32_t idx1) {
        m_data.line_indices.push_back(idx0);
        m_data.line_indices.push_back(idx1);
        m_data.line_indices.push_back(std::numeric_limits<uint32_t>::max()); // restart index
    }
private:
    draw_data<TPt>& m_data;
};


template <typename TMesh>
static uint32_t mesh_add_polygon(TMesh& mesh, 
    std::span<const osmpoint> verts, 
    std::span<const uint32_t> indices,
    double height, 
    bool reverse_vertices = false,
    bool reverse_winding = false)
{
    uint32_t vert_startidx = uint32_t(mesh.num_verts());

    if (reverse_vertices) {
        for (const auto& vert : verts | std::views::reverse) {
            mesh.add_vertex(vert.x, vert.y, height);
        }
    } else {
        for (const auto& vert : verts) {
            mesh.add_vertex(vert.x, vert.y, height);
        }
    }

    for (size_t i = 0; i < indices.size(); i += 3)
    {
        uint32_t idx0 = indices[i] + vert_startidx;
        uint32_t idx1 = indices[i + 1] + vert_startidx;
        uint32_t idx2 = indices[i + 2] + vert_startidx;

        if (reverse_winding) {
            mesh.add_triangle(idx0, idx2, idx1);
        } else {
            mesh.add_triangle(idx0, idx1, idx2);
        }
    }
    return vert_startidx;
}

static void gen_building_part_mesh(auto& mesh, const mesh_builder::building_part& part)
{
    bool reverse_verts = part.orient == ORIENT_CCW;
    auto tri_indices = polygon_triangulate(part.verts, reverse_verts);
    
    uint32_t bot_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_btm, reverse_verts, true);
    uint32_t top_verts_idx = mesh_add_polygon(mesh, part.verts, tri_indices, part.ht_top, reverse_verts);

    // print orientation of each face in the polygon
    //for (size_t i = 0; i < topbot_indices.size(); i += 3)
    //{
    //
    //    //int o = orient(part.coords[i], part.coords[(i + 1) % part.coords.size()], part.coords[(i + 2) % part.coords.size()]);
    //    //std::cout << o << "\n";
    //
    //    double o = orient(part.coords[topbot_indices[i]],
    //        part.coords[topbot_indices[(i + 1)]],
    //        part.coords[topbot_indices[(i + 2)]]);
    //    std::cout << o << "\n";
    //
    //    //size_t inext = (i + 1) % part.coords.size();
    //    //glm::dvec2 v1(part.coords[i].x, part.coords[i].y);
    //    //glm::dvec2 v2(part.coords[inext].x, part.coords[inext].y);
    //    //glm::dvec2 edge = v2 - v1;
    //    //glm::dvec2 normal(-edge.y, edge.x); // 90 degrees CCW rotation
    //    //double orientation = glm::dot(normal, glm::dvec2(0.0, 1.0)); // Y-axis is up
    //    //if (orientation > 0.0) {
    //    //    std::cout << "Face " << i << " is CCW\n";
    //    //} else if (orientation < 0.0) {
    //    //    std::cout << "Face " << i << " is CW\n";
    //    //} else {
    //    //    std::cout << "Face " << i << " is degenerate (collinear)\n";
    //    //}
    //}

    //boost::geometry::is_clo
    //std::cout << "\n";
    // bottom and top outlines
    //add_polyline(coords, min_height, part.is_closed);
    //add_polyline(coords, height, part.is_closed);

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

        mesh.add_triangle(quad[0], quad[2], quad[3]);
        mesh.add_triangle(quad[0], quad[3], quad[1]);

        //double o = orient(part.coords[quad[0]],
        //    part.coords[quad[2]],
        //    part.coords[quad[3]]);
        //std::cout << o << "\n";

        // outlines
        //mesh.add_segment(quad[0], quad[2]);
        //mesh.add_segment(quad[1], quad[3]);
    }
}

template <typename Kernel>
static draw_datad cgalmesh_draw_data(const cgalmesh<Kernel>& mesh, const std::string& name)
{
    draw_datad ret;
    ret.name = name;
    ret.verts.reserve(mesh.num_vertices() * 3);
    ret.tri_indices.reserve(mesh.num_faces() * 3);
    ret.line_indices.reserve(mesh.num_edges() * 3);

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

    for (auto e : mesh.edges()) 
    {
        auto h = mesh.halfedge(e);
        ret.line_indices.push_back(mesh.source(h).id());
        ret.line_indices.push_back(mesh.target(h).id());
        // opengl restart index
        ret.line_indices.push_back(std::numeric_limits<uint32_t>::max()); 
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
            } 
            while (h != start);
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

//namespace PMP = CGAL::Polygon_mesh_processing;
//using  K = CGAL::Exact_predicates_inexact_constructions_kernel;
//using  Point = K::Point_3;
//using  Mesh = CGAL::Surface_mesh<Point>;
//
//template <std::size_t N>
//Mesh make_mesh(const std::array<Point, N>& verts,
//    const std::vector<std::array<std::size_t, 3>>& tris)
//{
//    Mesh m;
//
//    /* 1.  Add all vertices and remember their indices */
//    std::vector<Mesh::Vertex_index> vmap;
//    vmap.reserve(verts.size());
//    for (const Point& p : verts)
//        vmap.push_back(m.add_vertex(p));
//
//    /* 2.  Add faces.  CGAL rejects degenerate or wrongly-oriented faces, so
//           make sure the winding is consistent and triangles are not flat.    */
//    for (const auto& t : tris)
//        m.add_face(vmap[t[0]], vmap[t[1]], vmap[t[2]]);
//
//    if (!CGAL::is_triangle_mesh(m))
//        throw std::runtime_error("Input did not form a valid closed triangle mesh");
//
//    return m;          // RVO – cheap
//}
//
//
//void test()
//{
//    // -------------------------------------------------------------------------
//// Example data: two unit cubes, the second one shifted so the cubes overlap
//// by half their size.  Replace these with your own data.
//    static const std::array<Point, 8> cubeA = { {
//      {0,0,0},{1,0,0},{1,1,0},{0,1,0},
//      {0,0,1},{1,0,1},{1,1,1},{0,1,1}
//    } };
//    static const std::array<Point, 8> cubeB = { {
//      {0.5,0.5,0.5},{1.5,0.5,0.5},{1.5,1.5,0.5},{0.5,1.5,0.5},
//      {0.5,0.5,1.5},{1.5,0.5,1.5},{1.5,1.5,1.5},{0.5,1.5,1.5}
//    } };
//
//    // 12 triangles per cube (two per face)
//    static const std::vector<std::array<std::size_t, 3>> cubeTris = {
//        /* bottom */ {0,1,2},{0,2,3},
//        /* top    */ {4,6,5},{4,7,6},
//        /* sides  */ {0,4,5},{0,5,1},
//                     {1,5,6},{1,6,2},
//                     {2,6,7},{2,7,3},
//                     {3,7,4},{3,4,0}
//    };
//
//    Mesh mesh1 = make_mesh(cubeA, cubeTris);
//    Mesh mesh2 = make_mesh(cubeB, cubeTris);
//
//    if (!CGAL::is_closed(mesh1) || !CGAL::is_closed(mesh2)) {
//        logERROR("L:IUe galusjvg");
//        return;
//    }
//
//    Mesh mesh_union;
//    bool ok = PMP::corefine_and_compute_union(mesh1, mesh2, mesh_union);
//
//    if (!ok)
//    {
//        std::cerr << "Union failed (most often means a mesh is not closed or self-intersects)\n";
//    }
//}

bool mesh_builder::add_building_drawdata(std::vector<draw_datad>& drawdata)
{
     aabb_tree<building*> bldg_tree;

    if (!m_building_parts.empty())
    {
        auto tree_ptrs = std::make_unique_for_overwrite<building*[]>(m_buildings.size());
        for (size_t i = 0; i < m_buildings.size(); ++i) {
            tree_ptrs[i] = &m_buildings[i];
        }
        bldg_tree = aabb_tree<building*>::create_unsafe(tree_ptrs.get(), m_buildings.size());
    }

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

        if (building.name.empty()) {
            logMESSAGE("Adding building %lld", building.info.id);
        } else {
            logMESSAGE("Adding building %lld (%s)", building.info.id, building.name.c_str());
        }

        if (building.parts.empty()) 
        {
            dd.name = building.name;
            drawdata_builder builder(dd);
            gen_building_part_mesh(builder, building.info);
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
        dd.name = "part " + std::to_string(part->id);

        drawdata_builder builder(dd);
        gen_building_part_mesh(builder, *part);

        drawdata.push_back(std::move(dd));
    }

    return true;
}

struct graph
{
#ifdef NDEBUG
    template <typename ...Args> using set_t = boost::container::flat_set<Args...>;
    template <typename ...Args> using map_t = boost::unordered::unordered_flat_map<Args...>;
#else
    // better VS debugging
    template <typename ...Args> using set_t = std::set<Args...>;
    template <typename ...Args> using map_t = std::unordered_map<Args...>;
#endif

    using edge_idpair = std::pair<osmium::object_id_type, osmium::object_id_type>;

    struct edge
    {
        const mesh_builder::street_way* way;
        bool visited;

        //osmium::object_id_type other_id(osmium::object_id_type id) const {
        //    assert(id == node_ids.first || id == node_ids.second);
        //    return node_ids.first == id ? node_ids.second : node_ids.first;
        //}
    };

    struct node
    {
        osmpoint vert;
        // Edge duplication is possible if two ways share segments, so use a set.
        // Two edges can also have the same way if a way loops back to 
        // its starting node.
        set_t<osmium::object_id_type> adj_node_ids;
    };

    // order-invariant
    struct edge_idpair_hash
    {
        std::size_t operator()(const edge_idpair& s) const noexcept
        {
            std::size_t seed = 0;
            auto minmax = std::minmax(s.first, s.second);
            boost::hash_combine(seed, minmax.first);
            boost::hash_combine(seed, minmax.second);
            return seed;
        }
    };
    // order-invariant
    struct edge_idpair_equals
    {
        bool operator()(const edge_idpair& lhs, const edge_idpair& rhs) const noexcept {
            return std::minmax(lhs.first, lhs.second) == std::minmax(rhs.first, rhs.second);
        }
    };

    map_t<osmium::object_id_type, node> nodes;
    map_t<edge_idpair, edge, edge_idpair_hash, edge_idpair_equals> edges;

    using node_itr = decltype(nodes)::iterator;
    using edge_itr = decltype(edges)::iterator;


    node_itr get_or_add_node(osmium::object_id_type id, osmpoint vert)
    {
        auto nodeitr = nodes.find(id);
        if (nodeitr == nodes.end())
        {
            graph::node node = { .vert = vert, .adj_node_ids = {} };
            nodeitr = nodes.insert({ id, std::move(node) }).first;
        }
        return nodeitr;
    }

    void add_edge(node_itr nodeitr, 
        osmium::object_id_type adj_node_id, 
        const mesh_builder::street_way* way)
    {
        nodeitr->second.adj_node_ids.insert(adj_node_id);

        edge_idpair idpair = { nodeitr->first, adj_node_id };
        if (!edges.contains(idpair))
        {
            graph::edge e = { .way = way, .visited = false };
            edges.insert({ idpair, std::move(e) });
        }
    }

    using fat_polyline = std::pair<std::vector<osmium::object_id_type>, double>;

    fat_polyline collect_polyline(node_itr nodeitr, osmium::object_id_type adj_nodeid, double eps = 1e-9)
    {
        std::vector<osmium::object_id_type> polyline;

        auto cur_nodeid = nodeitr->first;
        auto next_nodeid = adj_nodeid;
        const mesh_builder::street_way* prev_edgeway = nullptr;

        polyline.push_back(cur_nodeid);

        while (true)
        {
            graph::node_itr next_nodeitr = nodes.find(next_nodeid);
            if (next_nodeitr == nodes.end()) {
                break; // out of map bounds
            }

            graph::edge_itr edgeitr = edges.find({ cur_nodeid, next_nodeid });
            if (edgeitr == edges.end()) {
                logERROR("Missing graph edge between nodes %lld and %lld", cur_nodeid, next_nodeid); // change this to an assert at some point
                assert(false);
            }
            auto* cur_edgeway = edgeitr->second.way;

            bool can_visit_edge = !edgeitr->second.visited &&
                (!prev_edgeway || cur_edgeway == prev_edgeway ||
                    std::abs(cur_edgeway->width - prev_edgeway->width) < eps);

            if (!can_visit_edge) {
                break;
            }

            polyline.push_back(next_nodeid);
            edgeitr->second.visited = true;
            prev_edgeway = cur_edgeway;

            auto& next_node_adj_ids = next_nodeitr->second.adj_node_ids;
            if (next_node_adj_ids.size() > 2) {
                break; // stop at intersections
            }

            auto backup_cur_nodeid = cur_nodeid;
            cur_nodeid = next_nodeid;

            if (next_node_adj_ids.size() == 2) {
                next_nodeid = *ranges::find_if(next_node_adj_ids, [&](auto id) { return id != backup_cur_nodeid; });
            }
            else if (next_node_adj_ids.size() == 1) {
                next_nodeid = -1;
            }
            else {
                logERROR("Adjacent node %lld has no adjacent nodes?", next_nodeid);
                assert(false);
            }
        }

        if (polyline.size() < 2) {
            return { {}, 0 };
        }
        else {
            assert(prev_edgeway);
            return { std::move(polyline), prev_edgeway->width };
        }
    }
};

// instead of working with ids, maybe I dynamically allocate upfront
// (i.e. convert all the node ids into node pointers)
// all the nodes and edges and then point them to each other via pointers?
// ok for cache coherency as well if all of them are put into a vector instead
// of individually dynamically allocated? (eq. to a buffer allocator)
// still need: given two node pointers, get the edge pointer corresponding to it

// cant create upfront due to node duplication. just go with what I have for now

bool mesh_builder::add_street_drawdata(std::vector<draw_datad>& drawdata)
{
    graph graph;

    for (const auto& way : m_streetways)
    {
        for (size_t i = 0; i < way.nodes.size(); ++i)
        {
            auto* prev_waynode = (i == 0) ? nullptr : &way.nodes[i - 1];
            auto* cur_waynode = &way.nodes[i];
            auto* next_waynode = (i == way.nodes.size() - 1) ? nullptr : &way.nodes[i + 1];

            auto nodeitr = graph.get_or_add_node(cur_waynode->id, cur_waynode->vert);

            if (prev_waynode) { graph.add_edge(nodeitr, prev_waynode->id, &way); } // this is not needed if prevnode belongs to this way _exclusively_
            if (next_waynode) { graph.add_edge(nodeitr, next_waynode->id, &way); }
        }
    }

    // test
    for (auto adjnodeid : graph.nodes[25768772].adj_node_ids)
    {
        auto way_id = graph.edges[{ 25768772, adjnodeid }].way->id;
        logMESSAGE("node: %lld, way: %lld", adjnodeid, way_id);
    }

    constexpr double eps = 1e-9;

    
    std::vector<graph::fat_polyline> polylines;

    for (auto nodeitr = graph.nodes.begin(); nodeitr != graph.nodes.end(); ++nodeitr)
    {
        //if (nodeitr->first == 25768772)
        //{
            std::vector<graph::fat_polyline> node_polylines;

            for (auto adj_nodeid : nodeitr->second.adj_node_ids)
            {
                node_polylines.push_back(graph.collect_polyline(nodeitr, adj_nodeid, eps));
            }

            //logMESSAGE("node polylines size: %zu", node_polylines.size());
            
            draw_datad dd;
            drawdata_builder builder(dd);

            for (const auto& polyline : node_polylines)
            {
                if (polyline.first.size() == 0) {
                    continue;
                }

                uint32_t vert_startidx = uint32_t(builder.num_verts());

                for (auto id : polyline.first)
                {
                    auto pt = graph.nodes[id].vert;
                    builder.add_vertex(pt.x, pt.y, 0);
                }
                
                for (size_t i = 0; i < polyline.first.size() - 1; ++i)
                {
                    builder.add_line(i + vert_startidx, i + 1 + vert_startidx);
                }
            }

            drawdata.push_back(std::move(dd));
            //break;
        //}

        

        //for (auto adj_nodeid : node.adj_node_ids)
        //{
        //    while (!edge->visited) {
        //        
        //    }
        //}
    }

    // final: want to find all edges of same width that are joined together / adjacent, and not separated by intersections
    // why not at intersections? Because they must be handled separately.
    // so, I need two things: mark the intersections (so I know to stop at them), AND go both directions from a given node
    // Cannot just go from intersection to intersection because that will ignore closed loops in the graph!! (think: racetracks, private streets etc.)
    // intersections are simply those nodes that have more than two adjacent. That's it.
    // Need to de-duplicate the edges!
    // 
    // Don't need to handle case of intersection with 2 adjacent but diff width/way, because they will not be collected
    // anyway due to diff width. In fact if the width is the same, and 2 adjacent but diff streets, I WANT to collect it

    // start from any node which has only 2 adjacent.
    // Go in both directions collecting nodes as long as each has only 2 adjacent nodes
    // and don't belong to a way with different width.
    // Mark the edge to the adjacent node as visited.
    // Once stopped, this constitutes a polyline segment that can be buffered and drawn.
    // 
    // node should be marked as visited when it has been visited from every adjacent node
    // 
    // inner nodes can be marked as visited immediately. intersection nodes should
    // only be marked as visited when they have been visited from every adjacent node
    // 
    // What I really want is to mark graph edges as visited. If I have visited every edge,
    // I know I have all the polylines necessary to draw the streets
    // 



    logMESSAGE("Wait here!");
    return true;
}

std::vector<draw_datad> mesh_builder::get_draw_data()
{   
    std::vector<draw_datad> ret;

    add_building_drawdata(ret);
    add_street_drawdata(ret);

    return ret;
}

