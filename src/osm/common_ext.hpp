
#ifndef OSM_COMMON_EXPORT_HPP
#define OSM_COMMON_EXPORT_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <limits>

#include <glm/glm.hpp>


// Axis-aligned bounding box.
template <int N>
struct bbox
{
    using vec_t = glm::vec<N, double>;
    vec_t min;
    vec_t max;

    // Invalid by default, must be extended.
    constexpr bbox() :
        min(vec_t(std::numeric_limits<double>::infinity())),
        max(vec_t(-std::numeric_limits<double>::infinity()))
    {}

    constexpr bbox(const vec_t& min, const vec_t& max) :
        min(min), max(max)
    {
        assert(glm::all(glm::lessThanEqual(min, max)));
    }

    constexpr bbox(
        double min_x, double min_y, 
        double max_x, double max_y)
        requires (N == 2) :
        bbox(vec_t(min_x, min_y), vec_t(max_x, max_y))
    {}

    constexpr bbox(
        double min_x, double min_y, double min_z, 
        double max_x, double max_y, double max_z) 
        requires (N == 3) :
        bbox(vec_t(min_x, min_y, min_z), vec_t(max_x, max_y, max_z))
    {}

    static constexpr bbox infinity() {
        return {
            vec_t(-std::numeric_limits<double>::infinity()),
            vec_t(std::numeric_limits<double>::infinity())
        };
    }

    constexpr void extend(const bbox& other)
    {
        this->min = glm::min(this->min, other.min);
        this->max = glm::max(this->max, other.max);
    }

    constexpr void extend(const vec_t& point)
    {
        this->min = glm::min(this->min, point);
        this->max = glm::max(this->max, point);
    }

    constexpr vec_t center() const {
        return (min + max) / 2.0;
    }

    // Scale all dimensions by a factor, keeping center fixed.
    constexpr bbox<N> scaled(double scale) const
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

    constexpr bool intersects(const bbox& rhs) const {
        return
            glm::all(glm::lessThanEqual(this->min, rhs.max)) &&
            glm::all(glm::greaterThanEqual(this->max, rhs.min));
    }

    constexpr bool inside(const bbox& outer) const {
        return
            glm::all(glm::greaterThanEqual(this->min, outer.min)) &&
            glm::all(glm::lessThanEqual(this->max, outer.max));
    }

    constexpr bool contains(const vec_t& point) const {
        return
            glm::all(glm::greaterThanEqual(point, this->min)) &&
            glm::all(glm::lessThanEqual(point, this->max));
    }
};

// 2D axis-aligned bounding box
using bbox2d = bbox<2>;
// 3D axis-aligned bounding box
using bbox3d = bbox<3>;


enum osm_tri_type
{
    TRI_TYPE_GROUND,
    TRI_TYPE_BUILDING,
    TRI_TYPE_HIGHWAY,
    TRI_TYPE_WATER,
    NUM_TRI_TYPES
};

struct osm_gl_draw_data
{
    struct vertex
    {
        float pos[3];
        float normal[3];
    };
    struct tri {
        uint32_t idxs[3];
    };
    std::vector<vertex> verts;
    std::array<std::vector<tri>, NUM_TRI_TYPES> tris;
};


#endif