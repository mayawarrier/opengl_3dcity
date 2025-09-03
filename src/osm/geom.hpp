
#ifndef OSM_GEOM_HPP
#define OSM_GEOM_HPP

#include <utility>
#include <vector>
#include <limits>
#include <span>

#include "common.hpp"

#define GLM_ENABLE_EXPERIMENTAL 1
#include <glm/gtx/component_wise.hpp> 
#include <glm/gtc/constants.hpp>

using segment = std::pair<glm::dvec2, glm::dvec2>;

// Get a vector perpendicular to the input i.e. cross(z, vec).
inline glm::dvec2 vec_perp(glm::dvec2 vec) { return { -vec.y, vec.x }; }

// Get the squared length of a vector.
inline double vec_sqlength(glm::dvec2 vec) { return glm::dot(vec, vec); }

// Get the midpoint of a segment.
inline glm::dvec2 segment_mid(const segment& seg) {
    return (seg.first + seg.second) / 2.0;
}

// Get the midpoint of a segment defined by two indices.
inline glm::dvec2 segment_idx_mid(std::span<const glm::dvec2> points, size_t idx1, size_t idx2) {
    assert(idx1 < points.size() && idx2 < points.size());
    return (points[idx1] + points[idx2]) / 2.0;
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

// Get the minimum angle between two vectors (in radians).
double min_angle_between(glm::dvec2 a, glm::dvec2 b);

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

enum ray_bbox_inter_type
{
    RAYBOX_INTER_NONE = 0,
    RAYBOX_INTER_BORDER,
    RAYBOX_INTER_INSIDE
};

// Check if ray intersects a bounding box.
// If ray is normalized, t is distance.
template <int N>
ray_bbox_inter_type ray_intersects_bbox(const ray<N>& ray, 
    const bbox<N>& bbox, double& out_t, const param_range& t_range = {})
{
    glm::vec<N, double> tentries, texits;

    // intersect ray with every plane of the box
    for (int i = 0; i < N; ++i)
    {
        if (ray.dir[i] == 0) [[unlikely]]
        {
            // if not moving in this dim, must be within bounds
            if (ray.origin[i] < bbox.min[i] || ray.origin[i] > bbox.max[i]) {
                return RAYBOX_INTER_NONE;
            }
            tentries[i] = -std::numeric_limits<double>::infinity();
            texits[i] = std::numeric_limits<double>::infinity();
        }
        else {
            double tmin = (bbox.min[i] - ray.origin[i]) / ray.dir[i];
            double tmax = (bbox.max[i] - ray.origin[i]) / ray.dir[i];
            std::tie(tentries[i], texits[i]) = std::minmax(tmin, tmax);
        }
    }

    double tentry = glm::compMax(tentries);
    double texit = glm::compMin(texits);

    if (tentry > texit || tentry > t_range.max || texit < t_range.min) {
        return RAYBOX_INTER_NONE;
    }
    // t_entry <= t_range.max already checked
    bool on_border = tentry >= t_range.min; 

    out_t = on_border ? tentry : t_range.min;
    return on_border ? RAYBOX_INTER_BORDER : RAYBOX_INTER_INSIDE;
}

#endif