
#include <array>

#include <clipper2/clipper.h>
#include <mapbox/earcut.hpp>

#define GLM_ENABLE_EXPERIMENTAL 1
#include <glm/gtc/constants.hpp>

#include "geom.hpp"

static inline seg_inter_type classify_seg_inter_type(double param1, double param2)
{
    bool out1 = param1 < 0 || param1 > 1;
    bool out2 = param2 < 0 || param2 > 1;

    if (out1 && out2) { 
        return SEG_INTER_OUTSIDE_BOTH; 
    } else if (out1) { 
        return SEG_INTER_INSIDE_SEG2; 
    } else if (out2) { 
        return SEG_INTER_INSIDE_SEG1; 
    } else { 
        return SEG_INTER_INSIDE_BOTH; 
    }
}

// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
seg_inter_result seg_intersect(const segment& seg1, const segment& seg2, double eps)
{
    const glm::dvec2& a1 = seg1.first, &a2 = seg1.second;
    const glm::dvec2& b1 = seg2.first, &b2 = seg2.second;

    double denom = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    double t_numer = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x));
    double u_numer = ((a1.y - a2.y) * (a1.x - b1.x) - (a1.x - a2.x) * (a1.y - b1.y));

    if (std::abs(denom) < eps) 
    {
        if (std::abs(t_numer) < eps && std::abs(u_numer) < eps) {
            // proof: equate slope and y-intercept
            return { .type = SEG_INTER_COINCIDENT };
        } else {
            return { .type = SEG_INTER_PARALLEL };
        }
    }
    double t = t_numer / denom;
    double u = u_numer / denom;
    
    return {
        .type = classify_seg_inter_type(t, u),
        .point = a1 + t * (a2 - a1),
        .param_seg1 = t,
        .param_seg2 = u
    };
}

bool seg_proper_intersect(const segment& seg1, const segment& seg2, double eps)
{
    auto result = seg_intersect(seg1, seg2, eps);
    if (result.type == SEG_INTER_PARALLEL || result.type == SEG_INTER_COINCIDENT) {
        return false;
    }
    double t = result.param_seg1;
    double u = result.param_seg2;
    // exclude endpoints
    return (t > eps) && (t < 1.0 - eps) && (u > eps) && (u < 1.0 - eps);
}

seg_project_result seg_project_point(const segment& seg, glm::dvec2 point)
{
    glm::dvec2 ap = point - seg.first;
    glm::dvec2 ab = seg.second - seg.first;
    double t = glm::dot(ap, ab) / glm::dot(ab, ab);

    return {
        .proj = seg.first + t * ab,
        .proj_param = t,
        .is_inside = t >= 0 && t <= 1
    };
}

static inline double cos_angle_bw(const glm::dvec2& a, const glm::dvec2& b) {
    return glm::dot(a, b) / (glm::length(a) * glm::length(b));
}

double angle_bw(const glm::dvec2& a, const glm::dvec2& b)
{
    return std::acos(std::clamp(cos_angle_bw(a, b), -1.0, 1.0));
}

double angle_bw_unitvecs(const glm::dvec2& a, const glm::dvec2& b)
{
    return std::acos(std::clamp(glm::dot(a, b), -1.0, 1.0));
}

double acute_angle_bw(const glm::dvec2& a, const glm::dvec2& b)
{
    return std::acos(std::clamp(std::abs(cos_angle_bw(a, b)), 0.0, 1.0));
}

double acute_angle_bw_unitvecs(const glm::dvec2& a, const glm::dvec2& b)
{
    return std::acos(std::clamp(std::abs(glm::dot(a, b)), 0.0, 1.0));
}

glm::dvec2 rotate_vec2(const glm::dvec2& vec, double angle)
{
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    return {
        vec.x * cos_angle - vec.y * sin_angle,
        vec.x * sin_angle + vec.y * cos_angle
    };
}

static Clipper2Lib::PathsD get_clipper_poly(polygon_cspan in_poly, bbox2d& bbox)
{
    using namespace Clipper2Lib;
    
    PathsD clpoly;
    for (const auto& ring : in_poly) {
        PathD path;
        for (const auto& v : ring) {
            path.push_back(PointD{ v.x, v.y });
            bbox.extend(v);
        }
        clpoly.push_back(std::move(path));
    }
    return clpoly;
}

bool polygon_covered_by(polygon_cspan inner, polygon_cspan outer)
{
    using namespace Clipper2Lib;

    bbox2d bbox = bbox2d::empty();
    PathsD clip_path = get_clipper_poly(outer, bbox);
    PathsD subj_path = get_clipper_poly(inner, bbox);

    auto bb_center = bbox.center();
    for (auto& paths : { &subj_path, &clip_path }) {
        for (auto& path : *paths) {
            for (auto& pt : path) {
                pt.x -= bb_center.x;
                pt.y -= bb_center.y;
            }
        }
    }

    PathsD solution = Difference(subj_path, clip_path, FillRule::NonZero, 8);
    return solution.empty() || Area(solution[0]) < 1e-2;
}

// Earcut extension
namespace mapbox {
    namespace util {

        template <>
        struct nth<0, glm::dvec2> {
            inline static auto get(const glm::dvec2& t) {
                return t.x;
            };
        };
        template <>
        struct nth<1, glm::dvec2> {
            inline static auto get(const glm::dvec2& t) {
                return t.y;
            };
        };
    }
}

std::vector<uint32_t> polygon_triangulate(polygon_cspan polygon)
{
    return mapbox::earcut<uint32_t>(polygon);
}

bool check_triangles_oriented(std::span<const glm::dvec2> verts, std::span<const uint32_t> indices)
{
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        auto& v0 = verts[indices[i]];
        auto& v1 = verts[indices[i + 1]];
        auto& v2 = verts[indices[i + 2]];
        if (orient(v0, v1, v2) == ORIENT_CW) {
            return false;
        }
    }
    return true;
}

static void segment_triangulate(glm::dvec2 p0, glm::dvec2 p1, double width, draw_datad& dd)
{
    glm::dvec2 norm = (width / 2.0) * glm::normalize(vec_perp(p1 - p0));

    uint32_t vert_startidx = uint32_t(dd.num_verts());
    dd.add_vertex({ p0 - norm, 0.0 });
    dd.add_vertex({ p0 + norm, 0.0 });
    dd.add_vertex({ p1 + norm, 0.0 });
    dd.add_vertex({ p1 - norm, 0.0 });

    dd.add_triangle_w_offset(0, 3, 1, vert_startidx);
    dd.add_triangle_w_offset(3, 2, 1, vert_startidx);
}

struct stitch_edge
{
    orient_t orient;
    uint32_t inner_idx;
    uint32_t outer_idx;

    bool valid() const { 
        return inner_idx != UINT32_MAX && outer_idx != UINT32_MAX; 
    }
};

// Rare case when corner is really a straight line segment.
static stitch_edge degen_corner_triangulate(glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 p2,
    const stitch_edge& stitch_edge, double width, draw_datad& dd, double eps)
{
    glm::dvec2 norm = (width / 2.0) * glm::normalize(vec_perp(p1 - p0));

    uint32_t top_next_idx = dd.add_vertex({ p2 + norm, 0.0 });
    uint32_t bot_next_idx = dd.add_vertex({ p2 - norm, 0.0 });

    assert(!(stitch_edge.valid() && stitch_edge.orient == ORIENT_COLL));
    bool prev_is_ccw = !stitch_edge.valid() || stitch_edge.orient == ORIENT_CCW;

    uint32_t top_prev_idx, bot_prev_idx;
    if (stitch_edge.valid()) {
        // when CCW, prev inner is along normal, when CW, prev outer is along normal
        top_prev_idx = prev_is_ccw ? stitch_edge.inner_idx : stitch_edge.outer_idx;
        bot_prev_idx = prev_is_ccw ? stitch_edge.outer_idx : stitch_edge.inner_idx;
    } else {
        top_prev_idx = dd.add_vertex({ p0 + norm, 0.0 });
        bot_prev_idx = dd.add_vertex({ p0 - norm, 0.0 });
    }

    dd.add_triangle(top_prev_idx, bot_next_idx, top_next_idx);
    dd.add_triangle(bot_next_idx, top_prev_idx, bot_prev_idx);

    return {
        .orient = prev_is_ccw ? ORIENT_CCW : ORIENT_CW,
        .inner_idx = prev_is_ccw ? top_next_idx : bot_next_idx,
        .outer_idx = prev_is_ccw ? bot_next_idx : top_next_idx
    };
}

static stitch_edge corner_triangulate(glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 p2, 
    const stitch_edge& stitch_edge, double width, draw_datad& dd, double eps)
{
    glm::dvec2 norm1 = (width / 2.0) * glm::normalize(vec_perp(p1 - p0));
    glm::dvec2 norm2 = (width / 2.0) * glm::normalize(vec_perp(p2 - p1));

    // point towards outward bend
    orient_t orient = ::orient(p0, p1, p2);
    if (orient == ORIENT_COLL) {
        return degen_corner_triangulate(p0, p1, p2, stitch_edge, width, dd, eps);
    }
    else if (orient == ORIENT_CCW) {
        norm1 = -norm1;
        norm2 = -norm2;
    }
    
    glm::dvec2 inner_p0 = p0 - norm1, outer_p0 = p0 + norm1;
    glm::dvec2 inner_p2 = p2 - norm2, outer_p2 = p2 + norm2;
    glm::dvec2 outer_p1_seg1 = p1 + norm1, outer_p1_seg2 = p1 + norm2;

    auto inter_result = seg_intersect({ outer_p0, outer_p1_seg1 }, { outer_p2, outer_p1_seg2 }, eps);
    if (inter_result.type == SEG_INTER_PARALLEL || inter_result.type == SEG_INTER_COINCIDENT) {
        return degen_corner_triangulate(p0, p1, p2, stitch_edge, width, dd, eps);
    }

    glm::dvec2 norm_mid = inter_result.point - p1;
    glm::dvec2 inner_mid = p1 - norm_mid, outer_mid = inter_result.point;

    uint32_t inner_p0_idx, outer_p0_idx;
    if (stitch_edge.valid()) 
    {
        assert(stitch_edge.orient != ORIENT_COLL);
        inner_p0_idx = stitch_edge.inner_idx;
        outer_p0_idx = stitch_edge.outer_idx;
        if (stitch_edge.orient != orient) { 
            std::swap(inner_p0_idx, outer_p0_idx); 
        }
    } 
    else {
        inner_p0_idx = dd.add_vertex({ inner_p0, 0.0 });
        outer_p0_idx = dd.add_vertex({ outer_p0, 0.0 }); 
    }

    uint32_t outer_p1_seg1_idx = dd.add_vertex({ outer_p1_seg1, 0.0 });
    uint32_t inner_mid_idx     = dd.add_vertex({ inner_mid,     0.0 });   
    uint32_t outer_p1_seg2_idx = dd.add_vertex({ outer_p1_seg2, 0.0 });
    uint32_t inner_p2_idx      = dd.add_vertex({ inner_p2,      0.0 });
    uint32_t outer_p2_idx      = dd.add_vertex({ outer_p2,      0.0 });

    bool remove_mid_tri = angle_bw(outer_p1_seg2 - inner_mid, outer_p1_seg1 - inner_mid) < glm::radians(20.0);

    // Segment 1
    dd.add_triangle(inner_p0_idx, inner_mid_idx, outer_p0_idx);
    dd.add_triangle(outer_p0_idx, inner_mid_idx, (remove_mid_tri ? outer_p1_seg2_idx : outer_p1_seg1_idx));
    
    if (!remove_mid_tri) {
        uint32_t outer_mid_idx = dd.add_vertex({ outer_mid, 0.0 });
        dd.add_triangle(inner_mid_idx, outer_p1_seg2_idx, outer_p1_seg1_idx);
        dd.add_triangle(outer_p1_seg1_idx, outer_p1_seg2_idx, outer_mid_idx); // remove for bevelled corners
    }

    // Segment 2
    dd.add_triangle(inner_mid_idx, inner_p2_idx, outer_p2_idx);
    dd.add_triangle(inner_mid_idx, outer_p2_idx, outer_p1_seg2_idx);

    return { 
        .orient = orient, 
        .inner_idx = inner_p2_idx,
        .outer_idx = outer_p2_idx 
    };
}

// based on https://www.codeproject.com/Articles/226569/Drawing-polylines-by-tessellation
void polyline_triangulate(std::span<const glm::dvec2> polyline, double width, draw_datad& dd, double eps)
{
    if (polyline.size() < 2) {
        return;
    }

    if (polyline.size() == 2) {
        segment_triangulate(polyline[0], polyline[1], width, dd);
    }
    else {
        stitch_edge stitch_edge{ .inner_idx = UINT32_MAX, .outer_idx = UINT32_MAX };

        for (size_t i = 1; i < polyline.size() - 1; ++i)
        {
            glm::dvec2 p0 = (i == 1) ? polyline[0] : 
                ((polyline[i - 1] + polyline[i]) / 2.0);
            glm::dvec2 p2 = (i == polyline.size() - 2) ? polyline[i + 1] : 
                ((polyline[i] + polyline[i + 1]) / 2.0);

            stitch_edge = corner_triangulate(p0, polyline[i], p2, stitch_edge, width, dd, eps);
        }
    }
}
