
#include <boost/geometry.hpp>
#include <mapbox/earcut.hpp>

#include "geom.hpp"

// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
bool segments_proper_intersect(const osmsegment& seg1, const osmsegment& seg2)
{
    constexpr double eps = 1e-9;
    osmpoint a1 = seg1.first, a2 = seg1.second;
    osmpoint b1 = seg2.first, b2 = seg2.second;

    double denom = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    if (std::abs(denom) < eps) {
        return false;
    }
    double t = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x)) / denom;
    double u = ((a1.y - a2.y) * (a1.x - b1.x) - (a1.x - a2.x) * (a1.y - b1.y)) / denom;

    return (t > eps) && (t < 1.0 - eps) && (u > eps) && (u < 1.0 - eps);
}

bool polygon_covered_by(std::span<const osmpoint> inner, std::span<const osmpoint> outer)
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

            osmsegment segA{ outer[icurA], outer[inextA] };
            osmsegment segB{ inner[icurB], inner[inextB] };

            if (segments_proper_intersect(segA, segB)) {
                return false;
            }
        }
    }
    return true;
}

// https://en.wikipedia.org/wiki/Shoelace_formula
orient polygon_orient(std::span<const osmpoint> verts)
{
    double orient = 0.0;
    for (size_t icur = 0; icur < verts.size(); ++icur) 
    {
        size_t inext = (icur + 1) % verts.size();

        double term1 = verts[icur].y + verts[inext].y;
        double term2 = verts[icur].x - verts[inext].x;
        
        orient += term1 * term2;
    }

    if (orient > 0) {
        return ORIENT_CCW;
    } else if (orient < 0) {
        return ORIENT_CW;
    } else {
        return ORIENT_COLL;
    }
}

// Earcut extension
namespace mapbox {
    namespace util {

        template <>
        struct nth<0, osmpoint> {
            inline static auto get(const osmpoint& t) {
                return t.x;
            };
        };
        template <>
        struct nth<1, osmpoint> {
            inline static auto get(const osmpoint& t) {
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

std::vector<uint32_t> polygon_triangulate(std::span<const osmpoint> verts, bool reverse_orient)
{
    if (reverse_orient) {
        std::array<reversed_view<std::span<const osmpoint>>, 1> polygon = { { verts } };
        return mapbox::earcut<uint32_t>(polygon);
    } else {
        std::array<std::span<const osmpoint>, 1> polygon = { { verts } };
        return mapbox::earcut<uint32_t>(polygon);
    }
}