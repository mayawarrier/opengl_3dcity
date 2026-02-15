
#ifndef OSM_STREET_COMMON_HPP
#define OSM_STREET_COMMON_HPP

#include <vector>
#include <algorithm>
#include <utility>
#include <span>
#include <unordered_map>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/flat_set.hpp>

#include "../containers/way_network.hpp"
#include "../geom.hpp"
#include "../mesh.hpp"


using way_net = way_network<mesh_builder::highway>;

static inline int path_num_segs(const way_net::path* path) {
    return int(path->nodes.size()) - 1;
}

static inline segment path_seg(const way_net::path* path, int idx) {
    return { path->nodes[idx].vert(), path->nodes[idx + 1].vert() };
}

static inline double path_seg_width(const way_net::path* path, int idx) {
    // +1 because in_way is null for the first node
    return path->nodes[idx + 1].in_way->width;
}

static inline glm::dvec2 path_point(const way_net::path* path, int seg_idx, double seg_param) {
    return seg_at_param(path_seg(path, seg_idx), seg_param);
}

static inline bool path_has_node(const way_net::path* path, osmium::object_id_type id) {
    auto& c = path->nodes;
    return std::find_if(c.begin(), c.end(), [&](auto& n) { return n.id() == id; }) != c.end();
}

struct way_net_paths
{
    types::unord_flat_map<osmium::object_id_type, way_net::path*> path_map;
    std::vector<way_net::path> footpaths;
    std::vector<way_net::path> streets;
    // add more as needed
};

using aabb_bldg_tree = aabb_tree2d<mesh_builder::building*>;

#endif