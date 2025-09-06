
#ifndef OSM_COMMON_HPP
#define OSM_COMMON_HPP

#include <cstdint>
#include <vector>

#include <osmium/osm/types.hpp>
#include <glm/glm.hpp>

#ifdef NDEBUG
#include <boost/unordered/unordered_flat_map_fwd.hpp>
#include <boost/container/container_fwd.hpp>
#else
#include <unordered_map>
#include <set>
#endif

#include <boost/pool/poolfwd.hpp>

#include "../utils.hpp"

enum way_type
{
    WAY_TYPE_UNKNOWN,
    WAY_TYPE_STREET,
    WAY_TYPE_FOOTWAY
};

enum orient_t
{
    ORIENT_CW = -1,  // clockwise
    ORIENT_COLL = 0, // collinear
    ORIENT_CCW = 1,  // counter-clockwise
};

struct osm_node
{
    osmium::object_id_type id;
    glm::dvec2 vert;
};

namespace types
{
#ifdef NDEBUG
    template <typename ...Args> using unord_flat_map = boost::unordered::unordered_flat_map<Args...>;
    template <typename ...Args> using flat_set = boost::container::flat_set<Args...>;
#else
    // Better for debugging. Boost natvis files are not being picked up.
    template <typename ...Args> using unord_flat_map = std::unordered_map<Args...>; 
    template <typename ...Args> using flat_set = std::set<Args...>;
#endif

    template <typename T>
    using unsync_pool_alloc = boost::pool_allocator<T, 
        boost::default_user_allocator_new_delete, boost::details::pool::null_mutex>;
}

template <int N>
struct ray
{
    using vec_t = glm::vec<N, double>;
    vec_t origin;
    vec_t dir;

    ray reversed() const {
        return { origin, -dir };
    }
    vec_t at_point(double t) const {
        return origin + t * dir;
    }
};

using ray2d = ray<2>;
using ray3d = ray<3>;

struct param_range
{
    double min = 0.0;
    double max = std::numeric_limits<double>::infinity();
};

// Axis-aligned bounding box.
template <int N>
struct bbox
{
    using vec_t = glm::vec<N, double>;
    vec_t min;
    vec_t max;

    bbox() :
        min(std::numeric_limits<double>::infinity()),
        max(-std::numeric_limits<double>::infinity())
    {}

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
};

// 2D axis-aligned bounding box
using bbox2d = bbox<2>; 
// 3D axis-aligned bounding box
using bbox3d = bbox<3>; 

template <typename TVert>
struct draw_data
{
    std::string name;
    glm::vec4 color;
    std::vector<TVert> verts;
    std::vector<uint32_t> tri_indices; // GL_TRIANGLES

    uint32_t add_vertex(TVert x, TVert y, TVert z)
    {
        uint32_t idx = num_verts();
        verts.push_back(x);
        verts.push_back(y);
        verts.push_back(z);
        return idx;
    }

    uint32_t add_vertex(const glm::tvec3<TVert>& vert) {
        return add_vertex(vert.x, vert.y, vert.z);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        tri_indices.push_back(idx0);
        tri_indices.push_back(idx1);
        tri_indices.push_back(idx2);
    }

    void add_triangle_w_offset(uint32_t idx0, uint32_t idx1, uint32_t idx2, uint32_t offset) {
        add_triangle(idx0 + offset, idx1 + offset, idx2 + offset);
    }

    uint32_t num_verts() const { return uint32_t(verts.size() / 3); }
    uint32_t num_tris() const { return uint32_t(tri_indices.size() / 3); }

    draw_data<float> as_float() &&
    {
        draw_data<float> result;
        result.name = std::move(name);
        result.color = color;
        result.verts.reserve(verts.size());

        for (size_t i = 0; i < verts.size(); i += 3) {
            result.verts.push_back(float(verts[i]));
            result.verts.push_back(float(verts[i + 1]));
            result.verts.push_back(float(verts[i + 2]));
        }
        result.tri_indices = std::move(tri_indices);
        return result;
    }
};

using draw_dataf = draw_data<float>;
using draw_datad = draw_data<double>;



#endif