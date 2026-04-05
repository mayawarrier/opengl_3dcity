
#include "../geom/geom_boolean.hpp"
#include "../geom/geom_triangulation.hpp"

#include "geom_impl.hpp"

template double multipoly_coverage<osm_area::multipoly_t>(std::span<const osm_area::multipoly_t*>, const osm_area::multipoly_t&);

template bool multipoly_covered_by<osm_area::multipoly_t>(const osm_area::multipoly_t&, const osm_area::multipoly_t&, double);

template std::vector<uint32_t> polygon_triangulate<osm_area::poly_t>(const osm_area::poly_t&);