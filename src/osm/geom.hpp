
#ifndef OSM_GEOM_HPP
#define OSM_GEOM_HPP

#include <utility>
#include <vector>
#include <limits>
#include <span>

#include <glm/glm.hpp>

#include "../utils.hpp"


template <typename TVert>
struct draw_data
{
    std::string name;
    std::vector<TVert> verts;
    std::vector<uint32_t> tri_indices; // GL_TRIANGLES

    uint32_t add_vertex(double x, double y, double z) 
    {
        uint32_t idx = num_verts();
        verts.push_back(x);
        verts.push_back(y);
        verts.push_back(z);
        return idx;
    }

    uint32_t add_vertex(glm::dvec3 vert) {
        return add_vertex(vert.x, vert.y, vert.z);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        tri_indices.push_back(idx0);
        tri_indices.push_back(idx1);
        tri_indices.push_back(idx2);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2, uint32_t offset) {
        add_triangle(idx0 + offset, idx1 + offset, idx2 + offset);
    }

    uint32_t num_verts() const { return uint32_t(verts.size() / 3); }
};

using draw_dataf = draw_data<float>;
using draw_datad = draw_data<double>;

using segment = std::pair<glm::dvec2, glm::dvec2>;

// Get a vector perpendicular to the input i.e. cross(z, vec).
inline glm::dvec2 vec_perp(glm::dvec2 vec) { return { -vec.y, vec.x }; }

// Get the squared length of a vector.
inline double vec_sqlength(glm::dvec2 vec) { return glm::dot(vec, vec); }

// Get the midpoint of a segment.
inline glm::dvec2 segment_mid(const segment& seg) {
    return (seg.first + seg.second) / 2.0;
}

enum seg_inter_type
{
    INTER_PARALLEL,
    INTER_COINCIDENT,
    INTER_OUTSIDE_BOTH,
    INTER_INSIDE_SEG1,
    INTER_INSIDE_SEG2,
    INTER_INSIDE_BOTH
};

struct seg_inter_result
{
    // If type is PARALLEL or COINCIDENT, 
    // point and params are infinity
    seg_inter_type type;
    glm::dvec2 point;
    // Parametric coordinates on each segment
    double param_seg1;
    double param_seg2;

    seg_inter_result() = default;

    explicit seg_inter_result(seg_inter_type type) :
        type(type), 
        point(std::numeric_limits<double>::infinity()),
        param_seg1(std::numeric_limits<double>::infinity()),
        param_seg2(std::numeric_limits<double>::infinity())
    {}
};

// Intersect two line segments.
bool segments_intersect(const segment& seg1, const segment& seg2, seg_inter_result &out_result, double eps = 1e-9);

// Check for proper intersection of line segments 
// (i.e. not parallel/coinciding, and not at endpoints).
bool segments_proper_intersect(const segment& seg1, const segment& seg2, double eps = 1e-9);

// Get angle between two vectors (in radians).
double angle_between(glm::dvec2 a, glm::dvec2 b);

enum orient_t
{
    ORIENT_CW = -1,  // clockwise
    ORIENT_COLL = 0, // collinear
    ORIENT_CCW = 1,  // counter-clockwise
};

inline orient_t classify_orient(double value)
{
    if (value > 0) {
        return ORIENT_CCW;
    } else if (value < 0) {
        return ORIENT_CW;
    } else {
        return ORIENT_COLL;
    }
}

// Get orientation of points wrt to each other.
inline orient_t orient(glm::dvec2 a, glm::dvec2 b, glm::dvec2 c)
{
    glm::dvec2 v = b - a, w = c - a;
    return classify_orient(v.x * w.y - v.y * w.x);
}

// Get orientation of polygon.
orient_t polygon_orient(std::span<const glm::dvec2> polygon);

// Check if a polygon is within or on the border of another polygon.
bool polygon_covered_by(std::span<const glm::dvec2> inner_polygon, std::span<const glm::dvec2> outer_polygon);

// Triangulate polygon.
// Input and output are clockwise oriented.
// \param reverse_orient Reverse orientation of input vertices.
std::vector<uint32_t> polygon_triangulate(std::span<const glm::dvec2> polygon, bool reverse_orient = false);

// Triangulate a thick polyline.
void polyline_triangulate(std::span<const glm::dvec2> polyline, double width, draw_datad& dd, double eps = 1e-9);


// Axis-aligned bounding box.
struct bbox2d
{
    glm::dvec2 min;
    glm::dvec2 max;

    bbox2d() :
        min(std::numeric_limits<double>::infinity()),
        max(-std::numeric_limits<double>::infinity())
    {}

    void extend(const bbox2d& other)
    {
        this->min = glm::min(this->min, other.min);
        this->max = glm::max(this->max, other.max);
    }

    void extend(glm::dvec2 point)
    {
        this->min = glm::min(this->min, point);
        this->max = glm::max(this->max, point);
    }

    glm::dvec2 center() const {
        return (min + max) / 2.0;
    }

    bool intersects(const bbox2d& rhs) const
    {
        // Check if there is some overlap on the right 
        // (i.e. min before other box's max) and some overlap 
        // on the left (max after other box's min)
        return
            this->min.x <= rhs.max.x &&
            this->min.y <= rhs.max.y &&
            this->max.x >= rhs.min.x &&
            this->max.y >= rhs.min.y;
    }
};

#endif