
#include <unordered_map>
#include <algorithm>
#include <span>
#include <glm/glm.hpp>

#include <osmium/io/any_input.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include<CGAL/draw_surface_mesh.h>

#include <boost/geometry.hpp>
#include <mapbox/earcut.hpp>

#include "../utils.hpp"
#include "mesh.hpp"

namespace CGALPMP = CGAL::Polygon_mesh_processing;

namespace mapbox {
namespace util {

template <>
struct nth<0, osmium::geom::Coordinates> {
    inline static auto get(const osmium::geom::Coordinates& t) {
        return t.x;
    };
};
template <>
struct nth<1, osmium::geom::Coordinates> {
    inline static auto get(const osmium::geom::Coordinates& t) {
        return t.y;
    };
};
}}

// Check for proper intersection of line segments (i.e. not parallel, and not at endpoints).
// modified from https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
static bool segments_proper_intersect(
    const osmium::geom::Coordinates& a1, const osmium::geom::Coordinates& a2,
    const osmium::geom::Coordinates& b1, const osmium::geom::Coordinates& b2)
{
    constexpr double eps = 1e-9;

    double denom = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    if (std::abs(denom) < eps) {
        return false;
    }
    double t = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x)) / denom;
    double u = ((a1.y - a2.y) * (a1.x - b1.x) - (a1.x - a2.x) * (a1.y - b1.y)) / denom;

    return (t > eps) && (t < 1.0 - eps) && (u > eps) && (u < 1.0 - eps);
}

static bool polygon_contains(
    const std::vector<osmium::geom::Coordinates>& A,
    const std::vector<osmium::geom::Coordinates>& B)
{
    namespace bg = boost::geometry;
    using point_t = bg::model::d2::point_xy<double>;
    using polygon_t = bg::model::polygon<point_t>;

    polygon_t outer;
    for (size_t i = 0; i < A.size(); ++i) {
        bg::append(outer, point_t(A[i].x, A[i].y));
    }

    for (size_t i = 0; i < B.size(); ++i)
    {
        point_t pt(B[i].x, B[i].y);
        if (!bg::covered_by(pt, outer)) {
            return false;
        }
    }

    // Check for intersections between polygon edges.
    // Check only for proper intersections to avoid rejecting cases
    // where the inner polygon is on the border of the outer polygon - 
    // there are some weird cases where this will produce false positives
    // but they are unlikely.
    for (size_t icurB = 0; icurB < B.size(); ++icurB)
    {
        size_t inextB = (icurB + 1) % B.size();
        for (size_t icurA = 0; icurA < A.size(); ++icurA)
        {
            size_t inextA = (icurA + 1) % A.size();
            if (segments_proper_intersect(A[icurA], A[inextA], B[icurB], B[inextB])) {
                return false;
            }
        }
    }
    return true;
}

static osmium::geom::Coordinates calc_center(const osmium::NodeRefList& nr_list)
{
    osmium::geom::Coordinates c{ 0.0, 0.0 };
    for (const auto& nr : nr_list) {
        c.x += nr.lon();
        c.y += nr.lat();
    }

    c.x /= static_cast<double>(nr_list.size());
    c.y /= static_cast<double>(nr_list.size());
    return c;
}

template <typename T>
struct aabb_traits 
{
    bbox2d b;
    static const bbox2d& get_bbox(const T& obj) {
        (void)obj;
        return b;
    }
};

template <>
struct aabb_traits<building_assembler::building*>
{
    static const bbox2d& get_bbox(building_assembler::building* building) {
        return building->info.bbox;
    }
};

// Axis-aligned bounding box tree.
// Accelerates intersection queries.
template <typename T>
class aabb_tree
{
private:
    struct node
    {
        node* left;
        node* right;
        bbox2d bbox;
    };

    struct leafnode : public node
    {
        T data;
    };

public:
    aabb_tree() :
        m_root(nullptr)
    {}

    // Changes the order of the source array!
    static aabb_tree create_unsafe(T* objects, size_t num_objects) {
        return { objects, num_objects };
    }

    MOVE_ONLY_CLASS(aabb_tree, m_root, nullptr)

    std::vector<T> intersect(const bbox2d& bbox) const
    {
        std::vector<T> ret;
        std::vector<node*> candidates;

        auto insert_if_intersects = [&](node* node) {
            if (node && node->bbox.intersects(bbox))
                candidates.push_back(node);
        };

        insert_if_intersects(m_root);

        while (!candidates.empty())
        {
            node* node = candidates.back();
            candidates.pop_back();

            // If it intersects, descend further down the tree
            insert_if_intersects(node->left);
            insert_if_intersects(node->right);

            if (!node->left && !node->right) {
                auto* leaf = (leafnode*)node;
                ret.push_back(leaf->data);
            }
        }
        return ret;
    }

    ~aabb_tree() {
        delete_tree(m_root);
    }

private:
    aabb_tree(T* objects, size_t num_objects) :
        m_root(make_tree(objects, num_objects))
    {}

    static node* make_tree(T* objects, size_t num_objects)
    {
        if (num_objects == 0) {
            return nullptr;
        }
        else if (num_objects == 1)
        {
            auto* node = new leafnode();
            node->left = nullptr;
            node->right = nullptr;
            node->bbox = aabb_traits<T>::get_bbox(objects[0]);
            node->data = objects[0];
            return node;
        }
        else {
            auto* n = new node();

            for (size_t i = 0; i < num_objects; ++i) {
                n->bbox.extend(aabb_traits<T>::get_bbox(objects[i]));
            }

            glm::vec2 dim_sizes = n->bbox.max - n->bbox.min;
            int longest_dim = dim_sizes.x > dim_sizes.y ? 0 : 1;

            std::sort(objects, objects + num_objects,
                [&longest_dim](const T& lhs, const T& rhs) 
                {
                    double lhs_dim = aabb_traits<T>::get_bbox(lhs).center()[longest_dim];
                    double rhs_dim = aabb_traits<T>::get_bbox(rhs).center()[longest_dim];
                    return lhs_dim < rhs_dim;
                });

            size_t lhs_size = num_objects / 2; // truncated
            size_t rhs_size = num_objects - lhs_size;

            // Divide objects into half along longest dimension
            n->left = make_tree(objects, lhs_size);
            n->right = make_tree(objects + lhs_size, rhs_size);
            return n;
        }
    }

    static void delete_tree(node* node)
    {
        if (!node) { return; }

        delete_tree(node->left);
        delete_tree(node->right);

        if (!node->left && !node->right) {
            delete (leafnode*)node;
        } else {
            delete node;
        }
    }

private:
    node* m_root;
};


bool building_assembler::get_part(const osmium::Way& way, 
    double ht_bottom, double ht_top, part& out_part)
{
    auto& nodes = way.nodes();
    if (nodes.empty() || !nodes.is_closed()) {
        return false;
    }

    bbox2d bbox;
    std::vector<osmium::geom::Coordinates> coords;
    coords.resize(nodes.size() - 1);

    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        auto proj = osmium::geom::MercatorProjection{}(nodes[i].location());
        bbox.extend(glm::dvec2(proj.x, proj.y));
        coords[i] = proj;
    }

    out_part = {
        .id = way.id(),
        .bbox = bbox,
        .ht_btm = ht_bottom,
        .ht_top = ht_top,
        .coords = std::move(coords),
        .is_closed = nodes.is_closed()
    };
    return true;
}

void building_assembler::add_building(const osmium::Way& way,
    const char* name, bool is_part, double ht_bottom, double ht_top)
{
    part part;
    if (!get_part(way, ht_bottom, ht_top, part)) {
        logERROR("Failed to get part for way %d", way.id());
        return;
    }

    if (is_part) {
        m_parts.push_back(std::move(part));
    }
    else {
        m_buildings.push_back({
            .name = name ? name : "",
            .info = std::move(part),
            .parts = {},
        });
    }
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
//void add_polyline(const std::vector<osmium::geom::Coordinates>& verts, double height, bool is_closed)
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


template <typename TMesh>
uint32_t mesh_add_polygon(TMesh& mesh, 
    const std::vector<osmium::geom::Coordinates>& verts,
    const std::vector<uint32_t>& indices, 
    double height, bool reverse_winding = false)
{
    uint32_t vert_startidx = uint32_t(mesh.num_verts());

    for (const auto& vert : verts) {
        mesh.add_vertex(vert.x, vert.y, height);
    }
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        if (reverse_winding) {
            mesh.add_triangle(
                indices[i] + vert_startidx,
                indices[i + 2] + vert_startidx,
                indices[i + 1] + vert_startidx);
        }
        else {
            mesh.add_triangle(
                indices[i] + vert_startidx,
                indices[i + 1] + vert_startidx,
                indices[i + 2] + vert_startidx);
        }
    }
    return vert_startidx;
}

double cross(osmium::geom::Coordinates v, osmium::geom::Coordinates w) {
    return v.x * w.y - v.y * w.x; 
}

double orient(osmium::geom::Coordinates a, osmium::geom::Coordinates b, osmium::geom::Coordinates c) { 
    return cross(osmium::geom::Coordinates{ b.x - a.x, b.y - a.y }, osmium::geom::Coordinates{ c.x - a.x, c.y - a.y });
}

//bool isConvex(vector<pt> p) {
//    bool hasPos = false, hasNeg = false;
//    for (int i = 0, n = p.size(); i < n; i++) {
//        int o = orient(p[i], p[(i + 1) % n], p[(i + 2) % n]);
//        if (o > 0) hasPos = true;
//        if (o < 0) hasNeg = true;
//    }
//    return !(hasPos && hasNeg);
//}


template <typename TMesh>
static void mesh_add_building_part(TMesh& mesh, const building_assembler::part& part)
{
    //std::array<std::span<const osmium::geom::Coordinates>, 1> earcut_polylines;
    //earcut_polylines[0] = part.coords;

    std::vector<std::vector<osmium::geom::Coordinates>> earcut_polylines;
    earcut_polylines.resize(1);

    for (size_t i = 0; i < part.coords.size(); ++i) {
        earcut_polylines[0].push_back(part.coords[i]);
    }

    // todo: sometimes the node coordinates are not clockwise!
    // will need to be reversed, to iterate in a consistent order. Or flip the triangles afterwards

    auto topbot_indices = mapbox::earcut<uint32_t>(earcut_polylines);

    uint32_t bot_verts_idx = mesh_add_polygon(mesh, part.coords, topbot_indices, part.ht_btm);
    uint32_t top_verts_idx = mesh_add_polygon(mesh, part.coords, topbot_indices, part.ht_top, true);

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
    std::cout << "\n";
    // bottom and top outlines
    //add_polyline(coords, min_height, part.is_closed);
    //add_polyline(coords, height, part.is_closed);

    // sides
    for (uint32_t icur = 0; icur < part.coords.size(); ++icur)
    {
        uint32_t inext = (icur + 1) % part.coords.size();

        uint32_t quad[4] = {
            bot_verts_idx + icur,
            bot_verts_idx + inext,
            top_verts_idx + icur,
            top_verts_idx + inext,
        };
        // faces
        mesh.add_triangle(quad[0], quad[3], quad[2]);
        mesh.add_triangle(quad[0], quad[1], quad[3]);

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
using CGALMesh = CGAL::Surface_mesh<typename Kernel::Point_3>;

template <typename Kernel>
using CGALPoint = typename Kernel::Point_3;


template <typename Kernel>
struct cgalmesh_builder
{
    using Mesh = CGALMesh<Kernel>;
    using Point = CGALPoint<Kernel>;

    cgalmesh_builder(Mesh& m) : m_mesh(m) {}

    void add_vertex(double x, double y, double z)
    {
        auto idx = m_mesh.add_vertex(Point(x, y, z));
        m_vmap.push_back(idx);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2)
    {
        m_mesh.add_face(m_vmap[idx0], m_vmap[idx1], m_vmap[idx2]);
    }

    size_t num_verts() const { return m_vmap.size(); }

private:
    Mesh& m_mesh;
    std::vector<typename Mesh::Vertex_index> m_vmap;
};


template <typename Kernel>
static draw_datad cgalmesh_draw_data(const CGALMesh<Kernel>& mesh)
{
    return {};
}

namespace PMP = CGAL::Polygon_mesh_processing;
using  K = CGAL::Exact_predicates_inexact_constructions_kernel;
using  Point = K::Point_3;
using  Mesh = CGAL::Surface_mesh<Point>;

template <std::size_t N>
Mesh make_mesh(const std::array<Point, N>& verts,
    const std::vector<std::array<std::size_t, 3>>& tris)
{
    Mesh m;

    /* 1.  Add all vertices and remember their indices */
    std::vector<Mesh::Vertex_index> vmap;
    vmap.reserve(verts.size());
    for (const Point& p : verts)
        vmap.push_back(m.add_vertex(p));

    /* 2.  Add faces.  CGAL rejects degenerate or wrongly-oriented faces, so
           make sure the winding is consistent and triangles are not flat.    */
    for (const auto& t : tris)
        m.add_face(vmap[t[0]], vmap[t[1]], vmap[t[2]]);

    if (!CGAL::is_triangle_mesh(m))
        throw std::runtime_error("Input did not form a valid closed triangle mesh");

    return m;          // RVO – cheap
}


void test()
{
    // -------------------------------------------------------------------------
// Example data: two unit cubes, the second one shifted so the cubes overlap
// by half their size.  Replace these with your own data.
    static const std::array<Point, 8> cubeA = { {
      {0,0,0},{1,0,0},{1,1,0},{0,1,0},
      {0,0,1},{1,0,1},{1,1,1},{0,1,1}
    } };
    static const std::array<Point, 8> cubeB = { {
      {0.5,0.5,0.5},{1.5,0.5,0.5},{1.5,1.5,0.5},{0.5,1.5,0.5},
      {0.5,0.5,1.5},{1.5,0.5,1.5},{1.5,1.5,1.5},{0.5,1.5,1.5}
    } };

    // 12 triangles per cube (two per face)
    static const std::vector<std::array<std::size_t, 3>> cubeTris = {
        /* bottom */ {0,1,2},{0,2,3},
        /* top    */ {4,6,5},{4,7,6},
        /* sides  */ {0,4,5},{0,5,1},
                     {1,5,6},{1,6,2},
                     {2,6,7},{2,7,3},
                     {3,7,4},{3,4,0}
    };

    Mesh mesh1 = make_mesh(cubeA, cubeTris);
    Mesh mesh2 = make_mesh(cubeB, cubeTris);

    if (!CGAL::is_closed(mesh1) || !CGAL::is_closed(mesh2)) {
        logERROR("L:IUe galusjvg");
        return;
    }

    Mesh mesh_union;
    bool ok = PMP::corefine_and_compute_union(mesh1, mesh2, mesh_union);

    if (!ok)
    {
        std::cerr << "Union failed (most often means a mesh is not closed or self-intersects)\n";
    }
}

std::vector<draw_datad> building_assembler::get_draw_data()
{    
    auto tree_ptrs = std::make_unique_for_overwrite<building*[]>(m_buildings.size());
    for (size_t i = 0; i < m_buildings.size(); ++i) {
        tree_ptrs[i] = &m_buildings[i];
    }

    auto tree = aabb_tree<building*>::create_unsafe(tree_ptrs.get(), m_buildings.size());

    std::vector<part*> unmapped_parts;
    for (size_t i = 0; i < m_parts.size(); ++i)
    {
        size_t bldg_index = SIZE_MAX;

        auto inter_bldgs = tree.intersect(m_parts[i].bbox);
        if (inter_bldgs.size() == 1) {
            bldg_index = 0;
        }
        else {
            for (size_t icand = 0; icand < inter_bldgs.size() - 1; ++icand) {
                if (polygon_contains(inter_bldgs[icand]->info.coords, m_parts[i].coords)) {
                    bldg_index = icand;
                    break;
                }
            }
            // Last, skip polygon containment check
            if (bldg_index == SIZE_MAX) {
                bldg_index = inter_bldgs.size() - 1;
            }
        }
        if (bldg_index != SIZE_MAX) {
            inter_bldgs[bldg_index]->parts.push_back(&m_parts[i]);
        } else {
            unmapped_parts.push_back(&m_parts[i]);
        }
    }

    if (!unmapped_parts.empty()) {
        logWARNING("%z building parts could not be " 
            "mapped to a building", unmapped_parts.size());
    }

    //test();



    // build meshes from the parts and union the parts

    std::vector<draw_datad> ret;
    for (auto& building : m_buildings)
    {
        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using Mesh = CGALMesh<Kernel>;
    
        Mesh mesh;
        cgalmesh_builder<Kernel> mesh_builder(mesh);
        mesh_add_building_part(mesh_builder, building.info);
    
        //if (!CGALPMP::is_outward_oriented(mesh)) {
        //    logERROR("not outward");
        //    return {};
        //}
    
        std::vector<Mesh::halfedge_index> border_edges;
        CGALPMP::extract_boundary_cycles(mesh, std::back_inserter(border_edges));
    
        std::cout << "Mesh has " << border_edges.size() << " boundary halfedges\n";
    
        for (Mesh::halfedge_index h : mesh.halfedges()) {
            if (mesh.is_border(h)) {
                auto source = mesh.point(mesh.source(h));
                auto target = mesh.point(mesh.target(h));
                std::cout << "Border edge from " << source << " to " << target << "\n";
            }
        }
    
        for (size_t i = 0; i < border_edges.size(); ++i) {
            std::cout << "Cycle " << i << ":\n";
    
            Mesh::halfedge_index start = border_edges[i];
            Mesh::halfedge_index h = start;
    
            do {
                auto p1 = mesh.point(mesh.source(h));
                auto p2 = mesh.point(mesh.target(h));
                std::cout << "  Edge from " << p1 << " to " << p2 << "\n";
                h = mesh.next(h);
            } while (h != start);
        }
    
        if (!CGAL::is_triangle_mesh(mesh) || !CGAL::is_closed(mesh)) {
            logERROR("Failed here for mesh");
            return {};
        }
    
        
    
        for (size_t i = 0; i < building.parts.size(); ++i) 
        {
            part& part = *building.parts[i];
    
            Mesh part_mesh;
            cgalmesh_builder<Kernel> part_mesh_builder(part_mesh);
            mesh_add_building_part(part_mesh_builder, part);
    
            if (!CGAL::is_triangle_mesh(part_mesh) || !CGAL::is_closed(part_mesh)) {
                logERROR("Failed here for part");
                return {};
            }
    
            Mesh result_mesh;
            if (!CGALPMP::corefine_and_compute_union(mesh, part_mesh, result_mesh)) {
                logERROR("Failed to join part %d to building %d (%s)",
                    part.id, building.info.id, building.name.c_str());
                return {};
            }
            mesh = std::move(result_mesh);
        }
    
        ret.push_back(cgalmesh_draw_data<Kernel>(mesh));
    }

    return ret;
}

