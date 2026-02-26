
#include <vector>
#include <algorithm>
#include <utility>
#include <span>
#include <osmium/osm/node_ref_list.hpp>

#include "containers/way_network.hpp"
#include "geom.hpp"
#include "mesh_builder.hpp"


// https://wiki.openstreetmap.org/wiki/Highways
enum highway_type
{
    HIGHWAY_TYPE_UNKNOWN,
    HIGHWAY_TYPE_MOTORWAY,
    HIGHWAY_TYPE_TRUNK,
    HIGHWAY_TYPE_PRIMARY,
    HIGHWAY_TYPE_SECONDARY,
    HIGHWAY_TYPE_TERTIARY,
    HIGHWAY_TYPE_UNCLASSIFIED,
    HIGHWAY_TYPE_RESIDENTIAL,
    HIGHWAY_TYPE_MOTORWAY_LINK,
    HIGHWAY_TYPE_TRUNK_LINK,
    HIGHWAY_TYPE_PRIMARY_LINK,
    HIGHWAY_TYPE_SECONDARY_LINK,
    HIGHWAY_TYPE_TERTIARY_LINK,
    HIGHWAY_TYPE_LIVING_STREET,
    HIGHWAY_TYPE_SERVICE,
    HIGHWAY_TYPE_PEDESTRIAN,
    HIGHWAY_TYPE_ROAD,
    HIGHWAY_TYPE_FOOTWAY, // generic footway
    HIGHWAY_TYPE_FOOTWAY_SIDEWALK, // footway=sidewalk tag
    HIGHWAY_TYPE_FOOTWAY_CROSSING, // footway=crossing tag
};

// Type of highway based on its tags. 
// Returns -1 if not a highway and HIGHWAY_TYPE_UNKNOWN if 
// the highway tag is present but unrecognized.
int get_highway_type(const osmium::Way& way)
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
double get_highway_width(const osmium::Way& way, highway_type type)
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

bool mesh_builder::add_street_comp(const osmium::OSMObject* obj, mesh_object::comp_vec_t& comps)
{
    if (obj->type() == osmium::item_type::way)
    {
        auto* way = static_cast<const osmium::Way*>(obj);
        int type = get_highway_type(*way);
        if (type != -1) {
            comps.push_back({
                .type = mesh_component::COMP_TYPE_HIGHWAY,
                .subtype = type,
            });
            return true;
        }
    }
    // todo: handle areas
    return false;
}

struct way_net_paths
{
    types::unord_flat_map<osmium::object_id_type, way_net::path*> path_map;
    std::vector<way_net::path> footpaths;
    std::vector<way_net::path> streets;
    // add more as needed
};

static way_net_paths get_all_paths(way_net& network)
{
    way_net_paths ret;
    auto add_path = [&](way_net::path&& path) 
    {
        if (path.type == WAY_TYPE_FOOTWAY) {
            ret.footpaths.push_back(std::move(path));
        }
        else if (path.type == WAY_TYPE_STREET) {
            ret.streets.push_back(std::move(path));
        }
        else { assert_msg(false, "Unhandled path type %d", path.type); }
    };

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

                add_path(std::move(paths[0]));
            }
            else {
                if (collected[0]) { add_path(std::move(paths[0])); }
                if (collected[1]) { add_path(std::move(paths[1])); }
            }
        }
        else {
            for (auto adj_nodeid : adj_node_ids) {
                way_net::path path;
                if (network.path_to_intersection(nodeitr, adj_nodeid, visited_edges, path)) {
                    add_path(std::move(path));
                }
            }
        }
    }

    for (auto* pvec : { &ret.footpaths, &ret.streets }) {
        for (auto& path : *pvec) {
            ret.path_map[path.nodes.front().id()] = &path;
            ret.path_map[path.nodes.back().id()] = &path;
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

static bool gen_outline_drawdata(draw_datad& dd, const std::vector<outline_node>& outline)
{
    std::vector<glm::dvec2> outline_verts(outline.size());
    for (size_t i = 0; i < outline.size(); ++i) {
        outline_verts[i] = outline[i].vert;
    }
    if (path_orient(outline_verts) == ORIENT_CW) {
        std::reverse(outline_verts.begin(), outline_verts.end());
    }
    
    auto vert_span = std::span<const glm::dvec2>(outline_verts);
    auto tri_indices = polygon_triangulate(std::span(&vert_span, 1));
    if (tri_indices.empty()) {
        logDEBUG(LOG_MESSAGE, "Skipping outline since it has no triangles");
        return false;
    }
    assert(check_triangles_oriented(outline_verts, tri_indices));

    uint32_t vert_startidx = dd.num_verts();
    for (const auto& point : outline_verts) {
        dd.add_vertex(point.x, point.y, 0.0);
    }
    for (size_t i = 0; i < tri_indices.size(); i += 3) {
        dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
    }

    return true;
}

bool mesh_builder::gen_street_drawdata(std::vector<draw_datad>& drawdata, const aabb_tree2d<building*>* bldg_tree_ptr)
{
    constexpr double eps = 1e-9;

    logMESSAGE("-----------------------------------------------");
    logMESSAGE("Generating streets...");
    logMESSAGE("%zu ways, %zu nodes", m_highways.size(), m_num_highway_nodes);
    
    auto tbegin = clk::now();

    way_net network;
    timeit("Street network construction", [&]()
    {
        for (const auto& way : m_highways)
        {
            for (size_t i = 0; i < way.nodes.size(); ++i)
            {
                auto* prev_waynode = (i == 0) ? nullptr : &way.nodes[i - 1];
                auto* cur_waynode = &way.nodes[i];
                auto* next_waynode = (i == way.nodes.size() - 1) ? nullptr : &way.nodes[i + 1];

                auto nodeitr = network.get_or_add_node(cur_waynode->id, cur_waynode->vert);

                auto& adj_node_ids = nodeitr->second.adj_node_ids;
                if (prev_waynode) {
                    adj_node_ids.insert(prev_waynode->id);
                }
                if (next_waynode) {
                    adj_node_ids.insert(next_waynode->id);
                    network.add_edge({ nodeitr->first, next_waynode->id }, &way);
                }
            }
        }
    });
    
    way_net_paths all_paths;
    timeit("Path extraction", [&]() {
        all_paths = get_all_paths(network);
    });

    //auto st_outline_map = build_st_ft_outlines(network, all_paths, *bldg_tree_ptr, eps);
    street_outlines_t st_outline_map{};

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

    timeit("Outline triangulation", [&]()
    {
        for (auto& [street, outline] : st_outline_map) {
            if (!gen_outline_drawdata(street_dd, outline)) {
                gen_path_drawdata(street_dd, *street, eps);
            }
        }
    });

    timeit("Path triangulation", [&]()
    {
        //for (const auto& footpath : all_paths.footpaths) {
        //    gen_path_drawdata(footpath_dd, footpath, eps);
        //}
        for (const auto& street : all_paths.streets) {
            if (!st_outline_map.contains(&street)) {
                gen_path_drawdata(street_dd, street, eps);
            }
        }
    });

    uint32_t num_tris = street_dd.num_tris() + footpath_dd.num_tris();
    uint32_t num_verts = street_dd.num_verts() + footpath_dd.num_verts();

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Generated %u tris and %u vertices in %s", 
        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());

    return true;
}
