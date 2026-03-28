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

#include "../common.hpp"


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

// Get twice the signed area of a ring. 
// Positive if vertices are in CCW order, negative if CW, 0 if degenerate.
template <class TRing> requires Ring<TRing>
double ring_double_area(const TRing& verts)
{
    double ret = 0.0;
    for (size_t icur = 0; icur < verts.size(); ++icur)
    {
        size_t inext = (icur + 1) % verts.size();
        double term1 = vert_y(verts[icur]) + vert_y(verts[inext]);
        double term2 = vert_x(verts[icur]) - vert_x(verts[inext]);
        ret += term1 * term2;
    }
    return ret;
}

// Get signed area of a ring. 
// Positive if vertices are in CCW order, negative if CW, 0 if degenerate.
template <class TRing> requires Ring<TRing>
double ring_area(const TRing& verts)
{
    return ring_double_area(verts) / 2.0;
}

// Get signed area of a polygon.
template <class Poly> requires RingedPolygon<Poly>
double poly_area(const Poly& poly)
{
    double area = 0.0;
    for (const auto& ring : poly) {
        area += ring_area(ring);
    }
    return area;
}

// Get signed area of a multipolygon.
template <class MultiPoly> requires RingedMultiPolygon<MultiPoly>
double multipoly_area(const MultiPoly& mpoly)
{
    double area = 0.0;
    for (const auto& poly : mpoly) {
        area += poly_area(poly);
    }
    return area;
}

// Get orientation of a ring.
template <class TRing> requires Ring<TRing>
orient_t ring_orient(const TRing& verts)
{
    return orient_type(ring_double_area(verts));
}

// Check if triangles are oriented a certain way. 
// If allow_coll is true, then degenerate triangles are allowed.
bool check_tris_oriented(const auto& verts, 
    std::span<const uint32_t> indices, orient_t desired_orient, bool allow_coll = true)
{
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        auto v0 = vert_to_dvec2(verts[indices[i]]);
        auto v1 = vert_to_dvec2(verts[indices[i + 1]]);
        auto v2 = vert_to_dvec2(verts[indices[i + 2]]);
        
        auto o = orient(v0, v1, v2);
        if (o != desired_orient && !(allow_coll && o == ORIENT_COLL)) {
            return false;
        }
    }
    return true;
}

// Fraction of outer multipolygon covered by inner multipolygons.
// The inner multipolygons must be fully covered by the outer multipolygon.
// \return value in [0, 1].
template <class TMultiPoly> requires RingedMultiPolygon<TMultiPoly>
double multipoly_coverage(std::span<const TMultiPoly*> inner_mpolys, const TMultiPoly& outer_mpoly);

// Check if a multipolygon is inside another multipolygon.
// \param tol: max area of uncovered region in meters squared.
template <class TMultiPoly> requires RingedMultiPolygon<TMultiPoly>
bool multipoly_covered_by(const TMultiPoly& inner_mpoly, const TMultiPoly& outer_mpoly, double tol);

// Triangulate polygon.
template <class TPoly> requires RingedPolygon<TPoly>
std::vector<uint32_t> polygon_triangulate(const TPoly& polygon);

// Triangulate a thick polyline.
// todo: ideally agnostic of osm types
void polyline_triangulate(std::span<const glm::dvec2> polyline,
    double width, osm_tri_datad& dd, osm_tri_type tri_type, double eps = 1e-9);

// extern templates produce bogus VCR001 warnings, but there's no way to disable them. sigh
extern template double multipoly_coverage<osm_area::multipoly_t>(std::span<const osm_area::multipoly_t*>, const osm_area::multipoly_t&);
extern template bool multipoly_covered_by<osm_area::multipoly_t>(const osm_area::multipoly_t&, const osm_area::multipoly_t&, double);
extern template std::vector<uint32_t> polygon_triangulate<osm_area::poly_t>(const osm_area::poly_t&);

#endif
