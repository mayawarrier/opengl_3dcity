#ifndef OSM_GEOM_BOOLEAN_HPP
#define OSM_GEOM_BOOLEAN_HPP

#include <clipper2/clipper.h>

#include "geom.hpp"

enum boolean_op_type
{
    BOOL_OP_UNION,
    BOOL_OP_INTERSECTION,
    BOOL_OP_DIFFERENCE,
    BOOL_OP_XOR
};

template <MultiPaths MultiPoly>
void add_clipper_paths(Clipper2Lib::PathsD& paths, const MultiPoly& mpoly, bbox2d& bbox)
{
    using namespace Clipper2Lib;

    for (const auto& poly : mpoly) {
        for (const auto& ring : poly) {
            PathD path;
            for (const auto& v : ring) {
                path.push_back(PointD{ vert_x(v), vert_y(v) });
            }
            paths.push_back(std::move(path));
        }
    }
    for (const auto& poly : mpoly) {
        auto& outer_ring = poly[0];
        for (const auto& v : outer_ring) {
            bbox.extend(glm::dvec2{ vert_x(v), vert_y(v) });
        }
    }
}

template <MultiPaths MultiPoly>
inline Clipper2Lib::PathsD get_clipper_paths(const MultiPoly& mpoly, bbox2d& bbox)
{
    Clipper2Lib::PathsD paths;
    add_clipper_paths(paths, mpoly, bbox);
    return paths;
}

inline void center_clipper_paths(Clipper2Lib::PathsD& paths, glm::dvec2 center)
{
    for (auto& path : paths) {
        for (auto& pt : path) {
            pt.x -= center.x;
            pt.y -= center.y;
        }
    }
}

template <MultiPaths MultiPoly>
bool check_multipoly_valid(const MultiPoly& mpoly)
{
    if (mpoly.empty()) {
        return false;
    }
    for (const auto& poly : mpoly)
    {
        if (poly.empty()) {
            return false;
        }
        if (ring_orient(poly[0]) != ORIENT_CCW) {
            return false;
        }
        for (size_t i = 1; i < std::ranges::size(poly); ++i) {
            if (ring_orient(poly[i]) != ORIENT_CW) {
                return false;
            }
        }
    }
    return true;
}

template <MultiPaths MultiPoly>
bool multipoly_covered_by(const MultiPoly& inner_mpoly, const MultiPoly& outer_mpoly, double tol)
{
    using namespace Clipper2Lib;

    assert(check_multipoly_valid(inner_mpoly));
    assert(check_multipoly_valid(outer_mpoly));

    bbox2d bbox;
    PathsD clip_paths = get_clipper_paths(outer_mpoly, bbox);
    PathsD subj_paths = get_clipper_paths(inner_mpoly, bbox);

    auto bb_center = bbox.center();
    center_clipper_paths(clip_paths, bb_center);
    center_clipper_paths(subj_paths, bb_center);

    // note: using NonZero here, polygons must have consistent winding
    PathsD solution = Difference(subj_paths, clip_paths, FillRule::NonZero, 8);
    return solution.empty() || Area(solution) < tol;
}

template <MultiPaths MultiPoly>
double multipoly_coverage(std::span<const MultiPoly*> inner_mpolys, const MultiPoly& outer_mpoly)
{
    using namespace Clipper2Lib;

    bbox2d bbox;

    PathsD inner_subj_paths;
    for (const auto* inner_mpoly : inner_mpolys)
    {
        assert(check_multipoly_valid(*inner_mpoly));
        add_clipper_paths(inner_subj_paths, *inner_mpoly, bbox);
    }
    assert(check_multipoly_valid(outer_mpoly));
    PathsD outer_paths = get_clipper_paths(outer_mpoly, bbox);

    auto bb_center = bbox.center();
    center_clipper_paths(inner_subj_paths, bb_center);
    center_clipper_paths(outer_paths, bb_center);

    // note: using NonZero here, polygons must have consistent winding
    PathsD inner_union = Union(inner_subj_paths, FillRule::NonZero, 8);
    PathsD covered = Intersect(inner_union, outer_paths, FillRule::NonZero, 8);

    return std::clamp(Area(covered) / Area(outer_paths), 0., 1.);
}

#endif