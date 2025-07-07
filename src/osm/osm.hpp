#ifndef OSM_HPP
#define OSM_HPP

#include "../utils.hpp"
#include "common.hpp"

struct color_range 
{
    uint32_t startidx;
    uint32_t count;
    glm::vec4 color;
};

struct osm_data
{
    draw_dataf data;
    std::vector<color_range> color_ranges;
};

bool read_osmfile(const fs::path& path, osm_data& out_data);

#endif