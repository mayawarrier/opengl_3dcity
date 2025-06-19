
#include <unordered_map>
#include <algorithm>
#include <span>
#include <concepts>
#include <ranges>

#include <glm/glm.hpp>

#include <osmium/io/xml_input.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include <boost/geometry.hpp>
#include <mapbox/earcut.hpp>

#include "../utils.hpp"
#include "mesh.hpp"


namespace CGALPMP = CGAL::Polygon_mesh_processing;

template <typename Kernel>
using cgalmesh = CGAL::Surface_mesh<typename Kernel::Point_3>;

template <typename Kernel>
using cgalpoint = typename Kernel::Point_3;

using osmpoint = osmium::geom::Coordinates;
using osmsegment = std::pair<osmpoint, osmpoint>;

template <typename T, typename U>
concept has_value_type = std::same_as<typename T::value_type, U>;


// Check for proper intersection of line segments (i.e. not parallel, and not at endpoints).
// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
static bool segments_proper_intersect(const osmsegment& seg1, const osmsegment& seg2)
{
    constexpr double eps = 1e-9;
    osmpoint a1 = seg1.first, a2 = seg1.second;
    osmpoint b1 = seg2.first, b2 = seg2.second;

    double denom = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    if (std::abs(denom) < eps) {
        return false;
    }
    double t = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x)) / denom;
    double u = ((a1.y - a2.y) * (a1.x - b1.x) - (a1.x - a2.x) * (a1.y - b1.y)) / denom;

    return (t > eps) && (t < 1.0 - eps) && (u > eps) && (u < 1.0 - eps);
}

static bool polygon_covered_by(const std::vector<osmpoint>& inner, const std::vector<osmpoint>& outer)
{
    namespace bg = boost::geometry;
    using point_t = bg::model::d2::point_xy<double>;
    using polygon_t = bg::model::polygon<point_t>;

    polygon_t bg_outer;
    for (size_t i = 0; i < outer.size(); ++i) {
        bg::append(bg_outer, point_t(outer[i].x, outer[i].y));
    }

    for (size_t i = 0; i < inner.size(); ++i) {
        if (!bg::covered_by(point_t(inner[i].x, inner[i].y), bg_outer)) {
            return false;
        }
    }

    // Check for intersections between polygon edges.
    // Check only for proper intersections to avoid rejecting cases
    // where the inner polygon is on the border of the outer polygon - 
    // there are some weird cases where this will produce false positives
    // but they are unlikely.
    for (size_t icurB = 0; icurB < inner.size(); ++icurB)
    {
        size_t inextB = (icurB + 1) % inner.size();
        for (size_t icurA = 0; icurA < outer.size(); ++icurA)
        {
            size_t inextA = (icurA + 1) % outer.size();

            osmsegment segA{ outer[icurA], outer[inextA] };
            osmsegment segB{ inner[icurB], inner[inextB] };

            if (segments_proper_intersect(segA, segB)) {
                return false;
            }
        }
    }
    return true;
}

// https://en.wikipedia.org/wiki/Shoelace_formula
template <typename T> 
    requires has_value_type<T, osmpoint>
static orient polygon_orient(const T& coords)
{
    double orient = 0.0;
    for (size_t icur = 0; icur < coords.size(); ++icur) 
    {
        size_t inext = (icur + 1) % coords.size();

        double term1 = coords[icur].y + coords[inext].y;
        double term2 = coords[icur].x - coords[inext].x;
        orient += term1 * term2;
    }

    if (orient > 0) {
        return ORIENT_CCW;
    } else if (orient < 0) {
        return ORIENT_CW;
    } else {
        return ORIENT_COLL;
    }
}

//static osmpoint calc_center(const osmium::NodeRefList& nr_list)
//{
//    osmpoint c{ 0.0, 0.0 };
//    for (const auto& nr : nr_list) {
//        c.x += nr.lon();
//        c.y += nr.lat();
//    }
//
//    c.x /= static_cast<double>(nr_list.size());
//    c.y /= static_cast<double>(nr_list.size());
//    return c;
//}

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
    std::vector<osmpoint> coords;
    coords.resize(nodes.size() - 1);

    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
        auto proj = osmium::geom::MercatorProjection{}(nodes[i].location());
        bbox.extend(glm::dvec2(proj.x, proj.y));
        coords[i] = proj;
    }

    out_part = {
        .id = way.id(),
        .orient = polygon_orient(coords),
        .bbox = bbox,
        .ht_btm = ht_bottom,
        .ht_top = ht_top,
        .coords = std::move(coords)
    };
    return true;
}

bool building_assembler::add_building(const osmium::Way& way,
    const char* name, bool is_part, double ht_bottom, double ht_top)
{
    part part;
    if (!get_part(way, ht_bottom, ht_top, part)) {
        logERROR("Failed to get part for way %d", way.id());
        return false;
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

    //void add_line(uint32_t idx0, uint32_t idx1) {
    //    strip_indices.push_back(idx0);
    //    strip_indices.push_back(idx1);
    //    strip_indices.push_back(std::numeric_limits<uint32_t>::max()); // restart index
    //}
private:
    draw_data<TPt>& m_data;
};


template <typename TMesh, typename TVerts, typename TIndices>
requires 
    has_value_type<TVerts, osmpoint> && 
    has_value_type<TIndices, uint32_t>
uint32_t mesh_add_polygon(TMesh& mesh, const TVerts& verts, 
    const TIndices& indices, double height, bool reverse_winding = false)
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

        if (reverse_winding) {
            mesh.add_triangle(idx0, idx2, idx1);
        } else {
            mesh.add_triangle(idx0, idx1, idx2);
        }
    }
    return vert_startidx;
}

// Earcut extension
namespace mapbox {
namespace util {

template <>
struct nth<0, osmpoint> {
    inline static auto get(const osmpoint& t) {
        return t.x;
    };
};
template <>
struct nth<1, osmpoint> {
    inline static auto get(const osmpoint& t) {
        return t.y;
    };
};
}}

// Presents a reversed view of a container without copying it.
// Can't use std::views::reverse() because that doesn't have container typedefs.
template <typename T>
class reversed_view
{
public:
    using value_type = typename T::value_type;
    using reference = typename T::reference;
    using const_reference = typename T::const_reference;
    using iterator = typename T::reverse_iterator;
    using const_iterator = typename T::const_reverse_iterator;
    using difference_type = typename iterator::difference_type;
    using size_type = typename T::size_type;

    reversed_view(const T& data) : 
        m_data(data) 
    {}

    size_type size() const { return m_data.size(); }

    bool empty() const { return begin() == end(); }

    const_iterator begin() const { return m_data.crbegin(); }
    const_iterator end() const { return m_data.crend(); }

    const_reference operator[](size_type pos) const { 
        return m_data[m_data.size() - pos - 1];
    }

private:
    const T& m_data;
};

static void gen_building_part_mesh(auto& mesh, const building_assembler::part& part)
{
    auto add_top_and_bottom =
        [&](auto& mesh, const auto& poly) -> std::pair<uint32_t, uint32_t>
        {
            auto tri_indices = mapbox::earcut<uint32_t>(poly);
            uint32_t bot_verts_idx = mesh_add_polygon(mesh, poly[0], tri_indices, part.ht_btm, true);
            uint32_t top_verts_idx = mesh_add_polygon(mesh, poly[0], tri_indices, part.ht_top);
            return { bot_verts_idx, top_verts_idx };
        };

    uint32_t bot_verts_idx, top_verts_idx;

    if (part.orient == ORIENT_CCW) {
        std::array<reversed_view<std::vector<osmpoint>>, 1> polygon = { { part.coords } };
        std::tie(bot_verts_idx, top_verts_idx) = add_top_and_bottom(mesh, polygon);
    } else {
        std::array<std::span<const osmpoint>, 1> polygon = { { part.coords } };
        std::tie(bot_verts_idx, top_verts_idx) = add_top_and_bottom(mesh, polygon);
    }

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

std::vector<draw_datad> building_assembler::get_draw_data()
{   
    aabb_tree<building*> tree;

    if (!m_parts.empty())
    {
        auto tree_ptrs = std::make_unique_for_overwrite<building*[]>(m_buildings.size());
        for (size_t i = 0; i < m_buildings.size(); ++i) {
            tree_ptrs[i] = &m_buildings[i];
        }
        tree = aabb_tree<building*>::create_unsafe(tree_ptrs.get(), m_buildings.size());
    }

    std::vector<part*> unmapped_parts;
    for (auto& part : m_parts)
    {
        if (part.orient == ORIENT_COLL) {
            logWARNING("Ignoring part %lld as it has 0 area", part.id);
            continue;
        }

        building* mapped_bldg = nullptr;
        auto inter_bldgs = tree.intersect(part.bbox);
        if (inter_bldgs.empty()) 
        {
            logWARNING("Part %lld does not intersect any buildings", part.id);
            unmapped_parts.push_back(&part);
            continue;
        }
        else {
            for (size_t icand = 0; icand < inter_bldgs.size() - 1; ++icand) {
                if (polygon_covered_by(part.coords, inter_bldgs[icand]->info.coords)) {
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

    std::vector<draw_datad> ret;

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
                part& part = *building.parts[i];
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

        ret.push_back(std::move(dd));
    }

    for (auto* part : unmapped_parts)
    {
        draw_datad dd;
        dd.name = "part " + std::to_string(part->id);

        drawdata_builder builder(dd);
        gen_building_part_mesh(builder, *part);

        ret.push_back(std::move(dd));
    }

    return ret;
}

