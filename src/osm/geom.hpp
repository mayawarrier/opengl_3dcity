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
inline glm::dvec2 vec_perp(glm::dvec2 vec) { return { -vec.y, vec.x }; }

// Get the squared length of a vector.
inline double vec_sqlength(glm::dvec2 vec) { return glm::dot(vec, vec); }

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
bool seg_intersect(const segment& seg1, const segment& seg2, seg_inter_result &out_result, double eps = 1e-9);

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

// Get a normal vector to a segment.
// dir_left = 90 degree CCW, dir_right = 90 degree CW.
glm::dvec2 seg_normal(const segment& seg, direction dir, double width);

// Get angle between two vectors (in radians).
double angle_bw(glm::dvec2 a, glm::dvec2 b);

// Get angle between two normalized vectors (in radians).
double angle_bw_unitvecs(glm::dvec2 a, glm::dvec2 b);

// Get minimum angle between two vectors (in radians).
double min_angle_bw(glm::dvec2 a, glm::dvec2 b);

// Get minimum angle between two normalized vectors (in radians).
double min_angle_bw_unitvecs(glm::dvec2 a, glm::dvec2 b);

// Get minimum angle between two segments (in radians).
inline double min_angle_bw_segs(const segment& seg1, const segment& seg2)
{
    glm::dvec2 dir1 = seg1.second - seg1.first;
    glm::dvec2 dir2 = seg2.second - seg2.first;
    return min_angle_bw(dir1, dir2);
}

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
// \param orient - orientation of input vertices. Must be CCW or CW.
std::vector<uint32_t> polygon_triangulate(std::span<const glm::dvec2> polygon, orient_t orient);

// Triangulate a thick polyline.
void polyline_triangulate(std::span<const glm::dvec2> polyline, double width, draw_datad& dd, double eps = 1e-9);

// Check if two bounding boxes intersect.
template <int N>
bool bbox_intersects_bbox(const bbox<N>& lhs, const bbox<N>& rhs)
{
    // Check if there is some overlap on the right 
    // (i.e. min before other box's max) and some overlap 
    // on the left (max after other box's min)
    return
        glm::all(glm::lessThanEqual(lhs.min, rhs.max)) &&
        glm::all(glm::greaterThanEqual(lhs.max, rhs.min));
}

#endif