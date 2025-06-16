#ifndef OSM_HPP
#define OSM_HPP

#include "../utils.hpp"

struct osm_data
{
    std::vector<float> verts;

    // GL_TRIANGLES
    std::vector<uint32_t> tri_indices;
    // GL_LINE_STRIP with prim restart index
    std::vector<uint32_t> line_indices;
};

bool read_osmfile(const fs::path& path, osm_data& out_data);

#endif