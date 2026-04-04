
#ifndef OSM_COMMON_HPP
#define OSM_COMMON_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <ranges>

#include <osmium/osm/types.hpp>
#include <osmium/osm/tag.hpp>
#include <glm/glm.hpp>

#ifdef NDEBUG
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/unordered/unordered_flat_set.hpp>
#include <boost/container/small_vector.hpp>
#include <boost/container/flat_set.hpp>
#else
#include <unordered_map>
#include <unordered_set>
#include <set>
#endif

#include "common_ext.hpp"
#include "../utils.hpp"


enum orient_t
{
    ORIENT_CW = -1,  // clockwise
    ORIENT_COLL = 0, // collinear
    ORIENT_CCW = 1,  // counter-clockwise
};

enum direction
{
    DIR_UNDEF = -1,
    DIR_LEFT = 0,
    DIR_RIGHT = 1,
};

namespace types
{
    // Use std library in debug mode because it works with Natvis.
    // Boost has Natvis support but it's not being picked up.
#ifdef NDEBUG
    template <class ...Args> 
    using unord_flat_map = boost::unordered::unordered_flat_map<Args...>;

    template <class T, class Compare = std::less<T>, class ...Args>
    using unord_flat_set = boost::unordered::unordered_flat_set<T, Compare, Args...>;
    
    template <class T, class Compare = std::less<T>, class ...Args>
    using flat_set = boost::container::flat_set<T, Compare, Args...>;

    template <class T, size_t N, class Compare = std::less<T>> // note: C2210
    using small_flat_set = boost::container::small_flat_set<T, N, Compare>;

    template <class T, size_t N, typename ...Args> 
    using small_vector = boost::container::small_vector<T, N, Args...>;

#else
    template <class ...Args> 
    using unord_flat_map = std::unordered_map<Args...>; 

    template <class T, class Compare = std::less<T>, class ...Args>
    using unord_flat_set = std::unordered_set<T, Compare, Args...>;

    template <class T, class Compare = std::less<T>, class ...Args>
    using flat_set = std::set<T, Compare>;

    template <class T, size_t N, class Compare = std::less<T>> 
    using small_flat_set = std::set<T, Compare>;

    template <class T, size_t N, class ...Args> 
    using small_vector = std::vector<T>;
#endif
}

using segment = std::pair<glm::dvec2, glm::dvec2>;

template <int N>
struct ray
{
    using vec_t = glm::vec<N, double>;
    vec_t origin;
    vec_t dir;

    ray reversed() const {
        return { origin, -dir };
    }
    vec_t at_param(double t) const {
        return origin + t * dir;
    }
};

using ray2d = ray<2>;
using ray3d = ray<3>;

// Range of parameter t. 
// These values may be infinite - check before using!
class param_range
{
public:
    param_range() : 
        m_min(0.0), m_max(std::numeric_limits<double>::infinity()),
        m_min2(0.0), m_max2(std::numeric_limits<double>::infinity())
    {}

    param_range(double min, double max) : 
        m_min(min), m_max(max) 
    {
        assert(min <= max);
        m_min2 = min * min;
        m_max2 = max * max;
    }

    double min() const { return m_min; }
    double max() const { return m_max; }
    double min2() const { return m_min2; }
    double max2() const { return m_max2; }

    bool nonneg() const {
        return m_min >= 0.0 && m_min <= m_max;
    }

    bool overlaps(double r_min, double r_max) const {
        return !(m_max < r_min || r_max < m_min);
    }
    bool overlaps2(double r_min2, double r_max2) const {
        return !(m_max2 < r_min2 || r_max2 < m_min2);
    }

private:
    double m_min, m_max;
    double m_min2, m_max2;

};

enum osm_obj_type
{
    OSM_OBJ_TYPE_NODE,
    OSM_OBJ_TYPE_WAY,
    OSM_OBJ_TYPE_AREA
};

struct osm_object 
{
    osmium::object_id_type id;

    osm_object() = default;
    osm_object(osmium::object_id_type id) : 
        id(id) 
    {}
};

struct osm_node : osm_object
{
    glm::dvec2 vert;

    osm_node() = default;
    osm_node(osmium::object_id_type id, const glm::dvec2& vert) : 
        osm_object(id), vert(vert) 
    {}
};

struct osm_way : osm_object
{
    std::vector<osm_node> nodes;

    osm_way() = default;

    template <class NodeVec>
    osm_way(osmium::object_id_type id, NodeVec&& nodes) : 
        osm_object(id), nodes(std::forward<NodeVec>(nodes)) 
    {}
};

struct osm_area : osm_object
{
    using poly_t = types::small_vector<std::span<const osm_node>, 1>;
    using multipoly_t = types::small_vector<poly_t, 1>;

    // Poly rings are spans into the nodes vector. 
    // Rings in the same polygon must be stored contiguously!
    std::vector<osm_node> nodes;
    multipoly_t polys;

    osm_area() = default;

    template <class NodeVec, class PolyVec>
    osm_area(osmium::object_id_type id, NodeVec&& nodes, PolyVec&& polys) : 
        osm_object(id), 
        nodes(std::forward<NodeVec>(nodes)), 
        polys(std::forward<PolyVec>(polys)) 
    {}

    inline bool is_multipoly() const {
        return polys.size() > 1;
    }

    // Get the nodes in all rings of a polygon.
    static std::span<const osm_node> poly_nodes(const poly_t& poly) {
        return std::span(&poly.front().front(), poly.back().data() + poly.back().size());
    }
};

inline double vert_x(const glm::dvec2& vert) { return vert.x; }
inline double vert_y(const glm::dvec2& vert) { return vert.y; }

inline double vert_x(const osm_node& n) { return n.vert.x; }
inline double vert_y(const osm_node& n) { return n.vert.y; }

template <class TVert>
glm::dvec2 vert_to_dvec2(const TVert& vert) {
    return { vert_x(vert), vert_y(vert) };
}

template <class TVert>
concept Vertex = requires(const TVert& vert)
{
    { vert_x(vert) } -> std::convertible_to<double>;
    { vert_y(vert) } -> std::convertible_to<double>;
};

template <class TRing>
concept Ring = 
    std::ranges::contiguous_range<TRing> && 
    Vertex<std::ranges::range_value_t<TRing>>;

template <class TPoly>
concept RingedPolygon = 
    std::ranges::contiguous_range<TPoly> && 
    Ring<std::ranges::range_value_t<TPoly>>;

template <class TMultiPoly>
concept RingedMultiPolygon = 
    std::ranges::contiguous_range<TMultiPoly> && 
    RingedPolygon<std::ranges::range_value_t<TMultiPoly>>;


struct osm_tri
{
    glm::u32vec3 vert_idxs;
    osm_tri_type type;
};

template <class TVert>
struct osm_tri_data
{
    uint32_t add_vertex(const glm::tvec3<TVert>& vert) {
        uint32_t idx = num_verts();
        m_verts.push_back(vert);
        return idx;
    }

    void add_triangle(glm::u32vec3 vert_idxs, osm_tri_type type) {
        m_tris.push_back({ vert_idxs, type });
    }

    void add_triangle_w_offset(glm::u32vec3 vert_idxs, uint32_t offset, osm_tri_type type) {
        m_tris.push_back({ vert_idxs + glm::u32vec3(offset), type });
    }

    template <class OtherTVert>
    void add_tridata(const osm_tri_data<OtherTVert>& other)
    {
        uint32_t vert_offset = num_verts();
        for (auto& vert : other.verts()) {
            add_vertex(glm::tvec3<TVert>(TVert(vert.x), TVert(vert.y), TVert(vert.z)));
        }
        for (auto& tri : other.tris()) {
            add_triangle_w_offset(tri.vert_idxs, vert_offset, tri.type);
        }
    }

    uint32_t num_verts() const { return uint32_t(m_verts.size()); }
    uint32_t num_tris() const { return uint32_t(m_tris.size()); }

    const std::vector<glm::tvec3<TVert>>& verts() const { return m_verts; }
    const std::vector<osm_tri>& tris() const { return m_tris; }

    struct positions
    {
        uint32_t nverts;
        uint32_t ntris;
    };
    positions position() const {
        return { num_verts(), num_tris() };
    }
    void rollback(const positions& pos) {
        assert(pos.nverts <= m_verts.size() && pos.ntris <= m_tris.size());
        m_verts.resize(pos.nverts);
        m_tris.resize(pos.ntris);
    }

private:
    std::vector<glm::tvec3<TVert>> m_verts;
    std::vector<osm_tri> m_tris;
};

using osm_tri_dataf = osm_tri_data<float>;
using osm_tri_datad = osm_tri_data<double>;

inline bool is_multipolygon(const osmium::TagList& tags) {
    const char* type = tags["type"];
    return type && (!std::strcmp(type, "multipolygon") || !std::strcmp(type, "boundary"));
}

inline bool is_underground(const osmium::TagList& tags)
{
    const char* tunnel = tags["tunnel"];
    const char* location = tags["location"];
    return (tunnel && !std::strcmp(tunnel, "yes")) || 
        (location && !std::strcmp(location, "underground"));
}

#endif