#ifndef OSM_HPP
#define OSM_HPP

#include "draw_data.hpp"

bool read_osmfile(const std::string& filepath_or_url, osm_gl_draw_data& out_data);

#endif