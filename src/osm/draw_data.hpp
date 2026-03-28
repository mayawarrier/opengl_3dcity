
#ifndef OSM_DRAWDATA_HPP
#define OSM_DRAWDATA_HPP

#include <cstdint>
#include <vector>
#include <array>

enum osm_tri_type
{
    TRI_TYPE_BUILDING,
    TRI_TYPE_HIGHWAY,
    NUM_TRI_TYPES
};

struct osm_gl_draw_data
{
    struct vertex
    {
        float pos[3];
        float normal[3];
    };
    struct tri {
        uint32_t idxs[3];
    };
    std::vector<vertex> verts;
    std::array<std::vector<tri>, NUM_TRI_TYPES> tris;
};

#endif