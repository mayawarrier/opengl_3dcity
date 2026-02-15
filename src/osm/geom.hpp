//
// todo:
// Stuff in here is a mix of 2D and N-D geometry utilities (mostly 2D).
// Needs to be cleaned up or at least labeled better.
//

#ifndef OSM_GEOM_HPP
#define OSM_GEOM_HPP

#include <utility>
#include <vector>
#include <limits>
#include <span>

#include "common.hpp"

// Get a vector perpendicular to the input i.e. cross(z, vec).
// Applies a 90 degree CCW rotation.
inline glm::dvec2 vec_perp(const glm::dvec2& vec) { return { -vec.y, vec.x }; }

// Get the squared length of a vector.
inline double vec_sqlength(const glm::dvec2& vec) { return glm::dot(vec, vec); }

// Get the index of the minimum element in a vector.
template <int N, typename T>
int vec_argmin(const glm::vec<N, T>& vec)
{
    int min_idx = 0;
    for (int i = 1; i < N; ++i) {
        if (vec[i] < vec[min_idx]) {
            min_idx = i;
        }
    }
    return min_idx;
}

// Get the index of the maximum element in a vector.
template <int N, typename T>
int vec_argmax(const glm::vec<N, T>& vec)
{
    int max_idx = 0;
    for (int i = 1; i < N; ++i) {
        if (vec[i] > vec[max_idx]) {
            max_idx = i;
        }
    }
    return max_idx;
}

enum seg_inter_type
{
    SEG_INTER_PARALLEL,
    SEG_INTER_COINCIDENT,
    SEG_INTER_OUTSIDE_BOTH,
    SEG_INTER_INSIDE_SEG1,
    SEG_INTER_INSIDE_SEG2,
    SEG_INTER_INSIDE_BOTH
};

struct seg_inter_result
{
    // If type is PARALLEL or COINCIDENT, point and params are infinity
    seg_inter_type type;
    // Intersection point
    glm::dvec2 point = glm::dvec2(std::numeric_limits<double>::infinity());
    // Parametric coordinates on each segment
    double param_seg1 = std::numeric_limits<double>::infinity();
    double param_seg2 = std::numeric_limits<double>::infinity();
};

// Intersect two line segments.
seg_inter_result seg_intersect(const segment& seg1, const segment& seg2, double eps = 1e-9);

// Check for proper intersection of line segments 
// (i.e. not parallel/coinciding, and not at endpoints).
bool seg_proper_intersect(const segment& seg1, const segment& seg2, double eps = 1e-9);

struct seg_project_result
{
    // Projected point.
    glm::dvec2 proj;
    // Parametric coordinate on the segment.
    double proj_param;
    // True if point lies on the segment.
    bool is_inside; 
};

// Get the projection of a point on a segment.
seg_project_result seg_project_point(const segment& seg, glm::dvec2 point);

// Get point on segment at param t.
inline glm::dvec2 seg_at_param(const segment& seg, double t) {
    return seg.first + t * (seg.second - seg.first);
}

// Get unoriented angle between two vectors (in radians).
// Returns value in [0, pi].
double angle_bw(const glm::dvec2& a, const glm::dvec2& b);

// Get unoriented angle between two normalized vectors (in radians).
// Returns value in [0, pi].
double angle_bw_unitvecs(const glm::dvec2& a, const glm::dvec2& b);

// Get minimum unoriented angle between two vectors (in radians).
// Returns value in [0, pi/2].
double acute_angle_bw(const glm::dvec2& a, const glm::dvec2& b);

// Get minimum unoriented angle between two normalized vectors (in radians).
// Returns value in [0, pi/2].
double acute_angle_bw_unitvecs(const glm::dvec2& a, const glm::dvec2& b);

// Rotate a 2D vector by angle (radians +ve CCW).
glm::dvec2 rotate_vec2(const glm::dvec2& vec, double angle);

inline orient_t orient_type(double value)
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
inline orient_t orient(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c)
{
    glm::dvec2 v = b - a, w = c - a;
    return orient_type(v.x * w.y - v.y * w.x);
}

// Get orientation of a path.
// https://en.wikipedia.org/wiki/Shoelace_formula
orient_t path_orient(std::span<const glm::dvec2> verts);

// First span is the outer ring, subsequent spans are inner rings (if any).
// Outer ring must be CCW, inner rings CW.
using polygon_cspan = std::span<std::span<const glm::dvec2>>;
using polygon_span = std::span<std::span<glm::dvec2>>;

// Check if a polygon is inside another polygon.
bool polygon_covered_by(polygon_cspan inner_poly, polygon_cspan outer_poly);

// Triangulate polygon.
std::vector<uint32_t> polygon_triangulate(polygon_cspan polygon);

// Check if triangles are oriented CCW or collinear.
bool check_triangles_oriented(std::span<const glm::dvec2> verts, std::span<const uint32_t> indices);

// Triangulate a thick polyline.
void polyline_triangulate(std::span<const glm::dvec2> polyline, double width, draw_datad& dd, double eps = 1e-9);


#endif