
#ifndef OSM_MESH_STREETS_IMPL_HPP
#define OSM_MESH_STREETS_IMPL_HPP

#include <vector>
#include <algorithm>
#include <utility>
#include <span>
#include <osmium/osm/node_ref_list.hpp>

#include "../containers/way_network.hpp"
#include "../geom/geom.hpp"
#include "../mesh_builder.hpp"

#include "highways.hpp"


// Type of highway based on its tags. 
// Returns -1 if not a highway and HIGHWAY_TYPE_UNKNOWN if 
// the highway tag is present but unrecognized.
int highway_comp_builder::get_highway_type(const osmium::Way& way)
{
    const char* highway = way.tags()["highway"];
    if (!highway) {
        return -1;
    }
    if (std::strcmp(highway, "motorway") == 0)       return HIGHWAY_TYPE_MOTORWAY;
    if (std::strcmp(highway, "trunk") == 0)          return HIGHWAY_TYPE_TRUNK;
    if (std::strcmp(highway, "primary") == 0)        return HIGHWAY_TYPE_PRIMARY;
    if (std::strcmp(highway, "secondary") == 0)      return HIGHWAY_TYPE_SECONDARY;
    if (std::strcmp(highway, "tertiary") == 0)       return HIGHWAY_TYPE_TERTIARY;
    if (std::strcmp(highway, "unclassified") == 0)   return HIGHWAY_TYPE_UNCLASSIFIED;
    if (std::strcmp(highway, "residential") == 0)    return HIGHWAY_TYPE_RESIDENTIAL;
    if (std::strcmp(highway, "motorway_link") == 0)  return HIGHWAY_TYPE_MOTORWAY_LINK;
    if (std::strcmp(highway, "trunk_link") == 0)     return HIGHWAY_TYPE_TRUNK_LINK;
    if (std::strcmp(highway, "primary_link") == 0)   return HIGHWAY_TYPE_PRIMARY_LINK;
    if (std::strcmp(highway, "secondary_link") == 0) return HIGHWAY_TYPE_SECONDARY_LINK;
    if (std::strcmp(highway, "tertiary_link") == 0)  return HIGHWAY_TYPE_TERTIARY_LINK;
    if (std::strcmp(highway, "living_street") == 0)  return HIGHWAY_TYPE_LIVING_STREET;
    if (std::strcmp(highway, "service") == 0)        return HIGHWAY_TYPE_SERVICE;
    if (std::strcmp(highway, "pedestrian") == 0)     return HIGHWAY_TYPE_PEDESTRIAN;
    if (std::strcmp(highway, "road") == 0)           return HIGHWAY_TYPE_ROAD;

    if (std::strcmp(highway, "footway") == 0)
    {
        const char* footway = way.tags()["footway"];
        if (footway && std::strcmp(footway, "sidewalk") == 0) return HIGHWAY_TYPE_FOOTWAY_SIDEWALK;
        if (footway && std::strcmp(footway, "crossing") == 0) return HIGHWAY_TYPE_FOOTWAY_CROSSING;
        return HIGHWAY_TYPE_FOOTWAY;
    }

    return HIGHWAY_TYPE_UNKNOWN;
}

// Width of a highway type in meters. Uses the width tag if present.
// Returns -1 if type is invalid or for highways with tag area=yes.
double highway_comp_builder::get_highway_width(const osmium::Way& way, highway_type type)
{
    if (way.tags().has_tag("area", "yes")) {
        return -1.0;
    }

    double width;
    switch (type)
    {
    case HIGHWAY_TYPE_MOTORWAY:         width = 18.0;  break;
    case HIGHWAY_TYPE_TRUNK:            width = 18.0;  break;
    case HIGHWAY_TYPE_PRIMARY:          width = 18.0;  break;
    case HIGHWAY_TYPE_SECONDARY:        width = 18.0;  break;
    case HIGHWAY_TYPE_TERTIARY:         width = 18.0;  break;
    case HIGHWAY_TYPE_MOTORWAY_LINK:    width = 12.0;  break;
    case HIGHWAY_TYPE_TRUNK_LINK:       width = 12.0;  break;
    case HIGHWAY_TYPE_PRIMARY_LINK:     width = 12.0;  break;
    case HIGHWAY_TYPE_SECONDARY_LINK:   width = 12.0;  break;
    case HIGHWAY_TYPE_TERTIARY_LINK:    width = 12.0;  break;
    case HIGHWAY_TYPE_UNCLASSIFIED:     width = 12.0;  break;
    case HIGHWAY_TYPE_RESIDENTIAL:      width = 12.0;  break;
    case HIGHWAY_TYPE_LIVING_STREET:    width = 12.0;  break;
    case HIGHWAY_TYPE_PEDESTRIAN:       width = 12.0;  break;
    case HIGHWAY_TYPE_SERVICE:          width = 7.0;   break;
    case HIGHWAY_TYPE_FOOTWAY:
    case HIGHWAY_TYPE_FOOTWAY_SIDEWALK:
    case HIGHWAY_TYPE_FOOTWAY_CROSSING: width = 1.3;   break;
    case HIGHWAY_TYPE_UNKNOWN:
    case HIGHWAY_TYPE_ROAD:             width = 7.0;   break;
    default:
        assert(false);
        return -1.0;
    }
    // roughly convert from carto's pixel-based widths
    return width * (50.0 / 58.0);
}

using way_net = way_network<highway_comp>;

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

static std::vector<way_net::path> get_all_paths(way_net& network)
{
    std::vector<way_net::path> ret;
    way_net::edge_map_t<bool> visited_edges;

    // traverse outwards from _all_ nodes instead of just intersections to ensure
    // that disconnected components (for eg. racetracks) are not missed
    for (auto nodeitr = network.nodes.begin(); nodeitr != network.nodes.end(); ++nodeitr)
    {
        auto& adj_node_ids = nodeitr->second.adj_node_ids;
        if (adj_node_ids.size() == 2)
        {
            int index = 0;
            bool collected[2] = { false, false };
            way_net::path paths[2];

            for (auto adj_nodeid : adj_node_ids) {
                collected[index] = network.path_to_intersection(nodeitr, adj_nodeid, visited_edges, paths[index]);
                index++;
            }
            // node is in the middle of a path, merge both sides
            if (collected[0] && collected[1] && paths[0].type == paths[1].type)
            {
                auto& path0_nodes = paths[0].nodes;
                auto& path1_nodes = paths[1].nodes;

                std::reverse(path0_nodes.begin(), path0_nodes.end());

                for (size_t i = 1; i < path0_nodes.size(); ++i) {
                    path0_nodes[i].in_way = path0_nodes[i - 1].in_way;
                }
                path0_nodes[0].in_way = nullptr;
                path1_nodes[0].in_way = path0_nodes.back().in_way;
                path0_nodes.pop_back();

                for (const auto& node : path1_nodes) {
                    path0_nodes.push_back(node);
                }

                ret.push_back(std::move(paths[0]));
            }
            else {
                if (collected[0]) { ret.push_back(std::move(paths[0])); }
                if (collected[1]) { ret.push_back(std::move(paths[1])); }
            }
        }
        else {
            for (auto adj_nodeid : adj_node_ids) {
                way_net::path path;
                if (network.path_to_intersection(nodeitr, adj_nodeid, visited_edges, path)) {
                    ret.push_back(std::move(path));
                }
            }
        }
    }

    return ret;
}

static void gen_path_drawdata(draw_datad& dd, const way_net::path& path, double eps)
{
    std::vector<glm::dvec2> verts(path.nodes.size());
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        verts[i] = path.nodes[i].vert();
    }
    // todo: use each way's width when triangulating the polyline
    polyline_triangulate(verts, path.nodes[1].in_way->width, dd, eps);
}

//static bool gen_outline_drawdata(draw_datad& dd, const std::vector<outline_node>& outline)
//{
//    std::vector<glm::dvec2> outline_verts(outline.size());
//    for (size_t i = 0; i < outline.size(); ++i) {
//        outline_verts[i] = outline[i].vert;
//    }
//    if (path_orient(outline_verts) == ORIENT_CW) {
//        std::reverse(outline_verts.begin(), outline_verts.end());
//    }
//    
//    auto vert_span = std::span<const glm::dvec2>(outline_verts);
//    auto tri_indices = polygon_triangulate(std::span(&vert_span, 1));
//    if (tri_indices.empty()) {
//        logDEBUG(LOG_MESSAGE, "Skipping outline since it has no triangles");
//        return false;
//    }
//    assert(check_triangles_oriented(outline, tri_indices));
//
//    uint32_t vert_startidx = dd.num_verts();
//    for (const auto& point : outline_verts) {
//        dd.add_vertex(point.x, point.y, 0.0);
//    }
//    for (size_t i = 0; i < tri_indices.size(); i += 3) {
//        dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
//    }
//
//    return true;
//}

bool highway_comp_builder::do_build_all(const osm_mesh_object_db* obj_db, 
    const std::vector<highway_comp>& highways, const std::vector<bldg_comp>& buildings, 
    std::vector<draw_datad>& out_drawdata)
{
    constexpr double eps = 1e-9;

    logMESSAGE("%zu ways, %zu nodes", highways.size(), m_num_hiway_nodes);
    
    auto tbegin = clk::now();

    way_net network;
    timeit("Network construction", [&]()
    {
        for (const auto& way_comp : highways)
        {
            auto& way = obj_db->get<osm_mesh_object>(way_comp.mesh_obj_idx);
            auto& way_osm = obj_db->get<osm_way>(way.osm_obj_idx);

            for (size_t i = 0; i < way_osm.nodes.size(); ++i)
            {
                auto* prev_way_node = (i == 0) ? nullptr : &way_osm.nodes[i - 1];
                auto* cur_way_node = &way_osm.nodes[i];
                auto* next_way_node = (i == way_osm.nodes.size() - 1) ? nullptr : &way_osm.nodes[i + 1];

                auto nodeitr = network.get_or_add_node(cur_way_node->id, cur_way_node->vert);

                auto& adj_node_ids = nodeitr->second.adj_node_ids;
                if (prev_way_node) {
                    adj_node_ids.insert(prev_way_node->id);
                }
                if (next_way_node) {
                    adj_node_ids.insert(next_way_node->id);
                    network.add_edge({ nodeitr->first, next_way_node->id }, &way_comp);
                }
            }
        }
    });
    
    std::vector<way_net::path> all_paths;
    timeit("Path extraction", [&]() {
        all_paths = get_all_paths(network);
    });

    draw_datad footpath_dd { 
        .name = "footpaths", 
        .color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f) 
    };
    draw_datad street_dd { 
        .name = "streets", 
        .color = glm::vec4(0.25f, 0.25f, 0.25f, 1.0f) 
    };
    //draw_datad debug_dd { 
    //    .name = "debug", 
    //    .color = glm::vec4(1.0f, 0.f, 0.f, 1.f) 
    //};

    //timeit("Outline triangulation", [&]()
    //{
    //    for (auto& [street, outline] : st_outline_map) {
    //        if (!gen_outline_drawdata(street_dd, outline)) {
    //            gen_path_drawdata(street_dd, *street, eps);
    //        }
    //    }
    //});

    timeit("Path triangulation", [&]()
    {
        //for (const auto& footpath : all_paths.footpaths) {
        //    gen_path_drawdata(footpath_dd, footpath, eps);
        //}
        for (const auto& street : all_paths) {
            gen_path_drawdata(street_dd, street, eps);
        }
    });

    uint32_t num_tris = street_dd.num_tris() + footpath_dd.num_tris();
    uint32_t num_verts = street_dd.num_verts() + footpath_dd.num_verts();

    //drawdata.push_back(std::move(footpath_dd));
    out_drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Generated %u tris and %u vertices in %s", 
        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());

    return true;
}

#endif