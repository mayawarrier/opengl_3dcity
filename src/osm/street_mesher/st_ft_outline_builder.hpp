
#ifndef OSM_STREET_MESHER_ST_FT_OUTLINE_BUILDER_HPP
#define OSM_STREET_MESHER_ST_FT_OUTLINE_BUILDER_HPP

#include "common.hpp"

struct outline_node
{
    // -1 if point is generated and not an OSM node.
    osmium::object_id_type id;
    glm::dvec2 vert;

    static outline_node osm(const osm_node& n) {
        return { .id = n.id, .vert = n.vert };
    }
};
using street_outlines_t = types::unord_flat_map<const way_net::path*, std::vector<outline_node>>;

// Generate street outlines by snapping to footpaths, 
// if they are within a certain distance and angular tolerance. 
street_outlines_t build_st_ft_outlines(const way_net& network, 
    const way_net_paths& all_paths, const aabb_bldg_tree& bldg_tree, double eps);

#endif