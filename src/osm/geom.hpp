
#ifndef OSM_GEOM_HPP
#define OSM_GEOM_HPP

#include <utility>
#include <vector>
#include <limits>
#include <span>

#include "../utils.hpp"
#include "common.hpp"

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

// Center a batch of drawdata wrt to their global center.
// Returns the bounding box of the centered data.
bbox3d center_drawdata_batch(std::span<draw_datad> batch);

#endif