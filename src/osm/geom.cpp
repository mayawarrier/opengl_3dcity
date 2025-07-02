
#include <boost/geometry.hpp>
#include <mapbox/earcut.hpp>

#include "geom.hpp"

static inline seg_inter_type classify_seg_inter_type(double param1, double param2)
{
    bool out1 = param1 < 0 || param1 > 1;
    bool out2 = param2 < 0 || param2 > 1;

    if (out1 && out2) { 
        return INTER_OUTSIDE_BOTH; 
    } else if (out1) { 
        return INTER_INSIDE_SEG2; 
    } else if (out2) { 
        return INTER_INSIDE_SEG1; 
    } else { 
        return INTER_INSIDE_BOTH; 
    }
}

// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
bool segments_intersect(const segment& seg1, const segment& seg2, seg_inter_result& out_result, double eps)
{
    const glm::dvec2& a1 = seg1.first, &a2 = seg1.second;
    const glm::dvec2& b1 = seg2.first, &b2 = seg2.second;

    double denom = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    double numer_t = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x));
    double numer_u = ((a1.y - a2.y) * (a1.x - b1.x) - (a1.x - a2.x) * (a1.y - b1.y));

    if (std::abs(denom) < eps) 
    {
        if (std::abs(numer_t) < eps && std::abs(numer_u) < eps) {
            // proof: equate slope and y-intercept
            out_result = seg_inter_result(INTER_COINCIDENT);
            return true;
        } else {
            out_result = seg_inter_result(INTER_PARALLEL);
            return false;
        }
    }
    double t = numer_t / denom;
    double u = numer_u / denom;
    
    out_result.point = a1 + t * (a2 - a1);
    out_result.param_seg1 = t;
    out_result.param_seg2 = u;
    out_result.type = classify_seg_inter_type(t, u);
    
    return out_result.type == INTER_INSIDE_BOTH;
}

bool segments_proper_intersect(const segment& seg1, const segment& seg2, double eps)
{
    seg_inter_result result;
    segments_intersect(seg1, seg2, result, eps);

    if (result.type == INTER_PARALLEL || result.type == INTER_COINCIDENT) {
        return false;
    }
    double t = result.param_seg1;
    double u = result.param_seg2;
    // exclude endpoints
    return (t > eps) && (t < 1.0 - eps) && (u > eps) && (u < 1.0 - eps);
}

bool polygon_covered_by(std::span<const glm::dvec2> inner, std::span<const glm::dvec2> outer)
{
    namespace bg = boost::geometry;
    using point_t = bg::model::d2::point_xy<double>;
    using polygon_t = bg::model::polygon<point_t>;

    polygon_t bg_outer;
    for (size_t i = 0; i < outer.size(); ++i) {
        bg::append(bg_outer, point_t(outer[i].x, outer[i].y));
    }

    for (size_t i = 0; i < inner.size(); ++i) {
        if (!bg::covered_by(point_t(inner[i].x, inner[i].y), bg_outer)) {
            return false;
        }
    }

    // Check for intersections between polygon edges.
    // Check only for proper intersections to avoid rejecting cases
    // where the inner polygon is on the border of the outer polygon - 
    // there are some weird cases where this will produce false positives
    // but they are unlikely.
    for (size_t icurB = 0; icurB < inner.size(); ++icurB)
    {
        size_t inextB = (icurB + 1) % inner.size();
        for (size_t icurA = 0; icurA < outer.size(); ++icurA)
        {
            size_t inextA = (icurA + 1) % outer.size();

            segment segA{ outer[icurA], outer[inextA] };
            segment segB{ inner[icurB], inner[inextB] };

            if (segments_proper_intersect(segA, segB)) {
                return false;
            }
        }
    }
    return true;
}

double angle_between(glm::dvec2 a, glm::dvec2 b)
{
    double cos_theta = glm::dot(a, b) / (glm::length(a) * glm::length(b));
    return std::acos(std::clamp(cos_theta, -1.0, 1.0));
}

// https://en.wikipedia.org/wiki/Shoelace_formula
orient_t polygon_orient(std::span<const glm::dvec2> verts)
{
    double orient = 0.0;
    for (size_t icur = 0; icur < verts.size(); ++icur) 
    {
        size_t inext = (icur + 1) % verts.size();
        double term1 = verts[icur].y + verts[inext].y;
        double term2 = verts[icur].x - verts[inext].x;       
        orient += term1 * term2;
    }
    return classify_orient(orient);
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

// Presents a reversed view of a container without copying it.
// Can't use std::views::reverse() because that doesn't have container typedefs.
template <typename T>
class reversed_view
{
public:
    using value_type = typename T::value_type;
    using const_reference = typename T::const_reference;
    using const_iterator = typename T::reverse_iterator;
    using difference_type = typename const_iterator::difference_type;
    using size_type = typename T::size_type;

    reversed_view(const T& data) :
        m_data(data)
    {}

    size_type size() const { return m_data.size(); }

    bool empty() const { return begin() == end(); }

    const_iterator begin() const { return m_data.rbegin(); }
    const_iterator end() const { return m_data.rend(); }

    const_reference operator[](size_type pos) const {
        return m_data[m_data.size() - pos - 1];
    }

private:
    const T& m_data;
};

std::vector<uint32_t> polygon_triangulate(std::span<const glm::dvec2> verts, bool reverse_orient)
{
    if (reverse_orient) {
        std::array<reversed_view<std::span<const glm::dvec2>>, 1> polygon = { { verts } };
        return mapbox::earcut<uint32_t>(polygon);
    } else {
        std::array<std::span<const glm::dvec2>, 1> polygon = { { verts } };
        return mapbox::earcut<uint32_t>(polygon);
    }
}

static void polyline_anchor(glm::dvec2 p0, glm::dvec2 p1, glm::dvec2 p2, double width, draw_datad& dd, double eps)
{
    glm::dvec2 norm1 = width * glm::normalize(vec_perp(p1 - p0));
    glm::dvec2 norm2 = width * glm::normalize(vec_perp(p2 - p1));

    // point towards outward bend
    if (orient(p0, p1, p2) == ORIENT_CCW) {
        norm1 = -norm1;
        norm2 = -norm2;
    }

    glm::dvec2 inner_p0 = p0 - norm1, outer_p0 = p0 + norm1;
    glm::dvec2 inner_p2 = p2 - norm2, outer_p2 = p2 + norm2;
    glm::dvec2 outer_p1_seg1 = p1 + norm1, outer_p1_seg2 = p1 + norm2;

    seg_inter_result inter_result;
    segments_intersect({ outer_p0, outer_p1_seg1 }, { outer_p2, outer_p1_seg2 }, inter_result, eps);

    glm::dvec2 norm_mid = inter_result.point - p1;
    glm::dvec2 inner_mid = p1 - norm_mid, outer_mid = inter_result.point;

    uint32_t vert_startidx = uint32_t(dd.num_verts());
    dd.add_vertex({ inner_p0, 0.0 });
    dd.add_vertex({ outer_p0, 0.0 });
    dd.add_vertex({ outer_p1_seg1, 0.0 });
    dd.add_vertex({ inner_mid, 0.0 });
    dd.add_vertex({ outer_mid, 0.0 });
    dd.add_vertex({ outer_p1_seg2, 0.0 });
    dd.add_vertex({ inner_p2, 0.0 });
    dd.add_vertex({ outer_p2, 0.0 });

    dd.add_triangle(0, 1, 3, vert_startidx);
    dd.add_triangle(1, 2, 3, vert_startidx);
    dd.add_triangle(3, 2, 5, vert_startidx);
    dd.add_triangle(2, 4, 5, vert_startidx); // remove for bevelled corners
    dd.add_triangle(3, 7, 6, vert_startidx);
    dd.add_triangle(3, 5, 7, vert_startidx);

    //if (glm::degrees(angle_between(t0, t1)) < 30.0)
    //{
    //
    //}
}

void polyline_triangulate(std::span<const glm::dvec2> polyline, double width, draw_datad& dd, double eps)
{
    if (polyline.size() < 2) {
        return;
    }

    if (polyline.size() == 2) 
    {
        auto& p0 = polyline[0], &p1 = polyline[1];
        glm::dvec2 norm = width * glm::normalize(vec_perp(p1 - p0));

        uint32_t vert_startidx = uint32_t(dd.num_verts());
        dd.add_vertex({ p0 - norm, 0.0 });
        dd.add_vertex({ p0 + norm, 0.0 });
        dd.add_vertex({ p1 + norm, 0.0 });
        dd.add_vertex({ p1 - norm, 0.0 });

        dd.add_triangle(0, 1, 3, vert_startidx);
        dd.add_triangle(3, 1, 2, vert_startidx);
    }
    else {
        for (size_t i = 1; i < polyline.size() - 1; ++i)
        {
            glm::dvec2 p0 = (i == 1) ? polyline[0] :
                segment_mid({ polyline[i - 1], polyline[i] });
            glm::dvec2 p2 = (i == polyline.size() - 2) ? polyline[i + 1] :
                segment_mid({ polyline[i], polyline[i + 1] });

            polyline_anchor(p0, polyline[i], p2, width, dd, eps);
        }
    }
}
