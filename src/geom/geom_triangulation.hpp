#ifndef OSM_GEOM_TRIANGULATION_HPP
#define OSM_GEOM_TRIANGULATION_HPP

#include <mapbox/earcut.hpp>

#include "geom.hpp"

// Earcut extension
namespace mapbox {
    namespace util
    {
        template <Vertex T>
        struct nth<0, T> {
            inline static auto get(const T& t) {
                return vert_x(t);
            }
        };
        template <Vertex T>
        struct nth<1, T> {
            inline static auto get(const T& t) {
                return vert_y(t);
            }
        };
    }
}

template <Paths Poly>
static const path_vertex_t<Poly>* poly_vert_ptr(const Poly& poly, uint32_t vert_idx)
{
    for (const auto& ring : poly) {
        if (vert_idx < ring.size()) {
            return &ring[vert_idx];
        }
        vert_idx -= uint32_t(ring.size());
    }
    assert_msg(false, "vert_idx %ud out of bounds", vert_idx);
    return nullptr;
}

// Check if triangles are wound a certain way. 
// If allow_coll is true, then degenerate triangles are allowed.
template <Paths Poly>
static bool check_tris_winding(const Poly& poly,
    std::span<const uint32_t> indices, orient_type winding, bool allow_coll = true)
{
    assert(indices.size() % 3 == 0);
    for (size_t i = 0; i < indices.size(); i += 3)
    {
        auto v0 = vert_to_dvec2(*poly_vert_ptr(poly, indices[i]));
        auto v1 = vert_to_dvec2(*poly_vert_ptr(poly, indices[i + 1]));
        auto v2 = vert_to_dvec2(*poly_vert_ptr(poly, indices[i + 2]));

        orient_type o = orient(v0, v1, v2);
        if (o != winding && !(allow_coll && o == ORIENT_COLL)) {
            return false;
        }
    }
    return true;
}

template <Paths Poly>
std::vector<uint32_t> polygon_triangulate(const Poly& polygon)
{
    auto ret = mapbox::earcut<uint32_t>(polygon);

    // Earcut says it always returns CW triangles, but this doesn't seem
    // to be true thanks to this bug: https://github.com/mapbox/earcut/issues/133
    // Check and correct winding if required.
    if (!ret.empty())
    {
        assert(ret.size() % 3 == 0);

        orient_type tris_orient = ORIENT_COLL;
        for (size_t i = 0; i < ret.size(); i += 3)
        {
            auto v0 = vert_to_dvec2(*poly_vert_ptr(polygon, ret[i]));
            auto v1 = vert_to_dvec2(*poly_vert_ptr(polygon, ret[i + 1]));
            auto v2 = vert_to_dvec2(*poly_vert_ptr(polygon, ret[i + 2]));

            orient_type o = orient(v0, v1, v2);
            if (o == ORIENT_COLL) {
                continue;
            }
            else {
                tris_orient = o;
                break;
            }
        }
        // check that all tris have the same winding (or are degenerate)
        assert(check_tris_winding(polygon, ret, tris_orient));

        if (tris_orient == ORIENT_CW) {
            for (size_t j = 0; j < ret.size(); j += 3) {
                std::swap(ret[j + 1], ret[j + 2]);
            }
        }
    }
    return ret;
}

#endif