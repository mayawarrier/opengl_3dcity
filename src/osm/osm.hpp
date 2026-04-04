#ifndef OSM_HPP
#define OSM_HPP

#include "common_ext.hpp"

bool read_osmfile(const std::string& filepath_or_url, 
    osm_gl_draw_data& out_data, const bbox2d& latlon_bounds = bbox2d::infinity());

#endif