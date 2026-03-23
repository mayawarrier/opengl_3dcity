
#ifndef OSM_COMMON_HPP
#define OSM_COMMON_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <ranges>

#include <osmium/osm/types.hpp>
#include <glm/glm.hpp>

#ifdef NDEBUG
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/small_vector.hpp>
#include <boost/container/flat_set.hpp>
#else
#include <unordered_map>
#include <set>
#endif

#include "drawdata.hpp"
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
    using flat_set = boost::container::flat_set<T, Compare, Args...>;

    template <class T, size_t N, class Compare = std::less<T>> // note: C2210
    using small_flat_set = boost::container::small_flat_set<T, N, Compare>;

    template <class T, size_t N, typename ...Args> 
    using small_vector = boost::container::small_vector<T, N, Args...>;

#else
    template <class ...Args> 
    using unord_flat_map = std::unordered_map<Args...>; 

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

// Axis-aligned bounding box.
template <int N>
struct bbox
{
    using vec_t = glm::vec<N, double>;
    vec_t min;
    vec_t max;

    // Invalid by default, must be extended.
    bbox() : 
        min(vec_t(std::numeric_limits<double>::infinity())), 
        max(vec_t(-std::numeric_limits<double>::infinity())) 
    {}

    bbox(const vec_t& min, const vec_t& max) : 
        min(min), max(max) 
    {
        assert(glm::all(glm::lessThanEqual(min, max)));
    }
    
    void extend(const bbox& other)
    {
        this->min = glm::min(this->min, other.min);
        this->max = glm::max(this->max, other.max);
    }

    void extend(const vec_t& point)
    {
        this->min = glm::min(this->min, point);
        this->max = glm::max(this->max, point);
    }

    vec_t center() const {
        return (min + max) / 2.0;
    }

    // Scale all dimensions by a factor, keeping center fixed.
    bbox<N> scaled(double scale) const
    {
        vec_t center = this->center();
        vec_t half_vec = (this->max - this->min) / 2.0;
        vec_t new_half_vec = half_vec * scale;
        return { center - new_half_vec, center + new_half_vec };
    }

    // Scale area by a factor, keeping center fixed.
    bbox<N> scaled_by_area(double area_scale) const {
        return scaled(std::sqrt(area_scale));
    }

    bool intersects(const bbox& rhs) const {
        return
            glm::all(glm::lessThanEqual(this->min, rhs.max)) &&
            glm::all(glm::greaterThanEqual(this->max, rhs.min));
    }

    bool inside(const bbox& outer) const {
        return 
            glm::all(glm::greaterThanEqual(this->min, outer.min)) &&
            glm::all(glm::lessThanEqual(this->max, outer.max));
    }
};

// 2D axis-aligned bounding box
using bbox2d = bbox<2>; 
// 3D axis-aligned bounding box
using bbox3d = bbox<3>; 


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
    // Most areas are single polygon + single ring, so optimize for this case.
    using poly_t = types::small_vector<std::span<const osm_node>, 1>;
    using multipoly_t = types::small_vector<poly_t, 1>;

    // Poly rings are spans into the nodes vector. 
    // Rings in the same polygon are always stored contiguously.
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

// First ring is the outer ring, subsequent rings are inner rings (if any).
// Outer ring must be CCW, inner rings CW.
template <class TPoly>
concept RingedPolygon = 
    std::ranges::contiguous_range<TPoly> && 
    Ring<std::ranges::range_value_t<TPoly>>;

template <class TMultiPoly>
concept RingedMultiPolygon = 
    std::ranges::contiguous_range<TMultiPoly> && 
    RingedPolygon<std::ranges::range_value_t<TMultiPoly>>;

#endif