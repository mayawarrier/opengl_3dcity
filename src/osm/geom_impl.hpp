#ifndef OSM_COMP_BUILDERS_GEOM_IMPL_HPP
#define OSM_COMP_BUILDERS_GEOM_IMPL_HPP

#include "../geom/geom.hpp"

extern template double multipoly_coverage<osm_area::multipoly_t>(std::span<const osm_area::multipoly_t*>, const osm_area::multipoly_t&);

extern template bool multipoly_covered_by<osm_area::multipoly_t>(const osm_area::multipoly_t&, const osm_area::multipoly_t&, double);

extern template std::vector<uint32_t> polygon_triangulate<osm_area::poly_t>(const osm_area::poly_t&);

#endif