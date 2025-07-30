
#include <algorithm>
#include <span>
#include <unordered_map>
#include <iterator>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include "../utils.hpp"
#include "way_network.hpp"

#include "mesh.hpp"


bool mesh_builder::add_highway(const highway_info& info)
{
    std::vector<way_node> nodes;
    for (const auto& nr : info.way.nodes)
    {
        auto proj = osmium::geom::MercatorProjection{}(nr.location());
        nodes.push_back({ nr.ref(), glm::dvec2(proj.x, proj.y) });
    }

    m_highways.push_back({
        .id = info.way.id,
        .name = info.way.name ? info.way.name : "",
        .type = info.type,
        .nodes = std::move(nodes),
        .width = info.width,
    });

    return true;
}

// instead of working with ids, maybe I dynamically allocate upfront
// (i.e. convert all the node ids into node pointers)
// all the nodes and edges and then point them to each other via pointers?
// ok for cache coherency as well if all of them are put into a vector instead
// of individually dynamically allocated? (eq. to a buffer allocator)
// still need: given two node pointers, get the edge pointer corresponding to it
// cant create upfront due to node duplication. just go with what I have for now

// final: want to find all edges of same width that are joined together / adjacent, and not separated by intersections
// why not at intersections? Because they must be handled separately.
// so, I need two things: mark the intersections (so I know to stop at them), AND go both directions from a given node
// Cannot just go from intersection to intersection because that will ignore closed loops in the graph!! (think: racetracks, private streets etc.)
// intersections are simply those nodes that have more than two adjacent. That's it.
// Need to de-duplicate the edges!
// 
// Don't need to handle case of intersection with 2 adjacent but diff width/way, because they will not be collected
// anyway due to diff width. In fact if the width is the same, and 2 adjacent but diff streets, I WANT to collect it

// start from any node which has only 2 adjacent.
// Go in both directions collecting nodes as long as each has only 2 adjacent nodes
// and don't belong to a way with different width.
// Mark the edge to the adjacent node as visited.
// Once stopped, this constitutes a polyline segment that can be buffered and drawn.
// 
// node should be marked as visited when it has been visited from every adjacent node
// 
// inner nodes can be marked as visited immediately. intersection nodes should
// only be marked as visited when they have been visited from every adjacent node
// 
// What I really want is to mark graph edges as visited. If I have visited every edge,
// I know I have all the polylines necessary to draw the streets
// 

// For the footpaths,
// go through all the street segments and assign parallel footpaths as their parent way
// Footpath may have more than one parent way in rare cases.
// PARALLEL = within a certain distance and angular tolerance
// from these footpaths, I can then generate the outlines of the streets
// if footpath is not present, I can use nearby buildings to estimate width and draw as polyline
// roads are unlikely to be perfectly flush with the buildings, so subtract 1 meter from the side with a building.
// This would need to done for each street segment separately (between two nodes)
// Width on one side of the street may be different from the other side, so I need to draw two outlines
// Polyline collection should happen not with width, but with street properties (name, type, lanes, etc.), and then
// estimate width for every segment of the collected polyline from nearby footpaths or buildings
// When width or angular difference becomes too large, cut the street at the point (this will happen at intersections)
// Whatever points are left over at the intersection after cutting, should be used to draw the intersection
// consider also: leaving a buffer zone of 10m around the intersection, so that the streets don't overlap

// Strategy:
// Snap the street segments to the footpaths, if they are within a certain distance and angular tolerance
// Consider left and right outlines of the street separately
// If footpath is not present, just draw the outline normally, by using estimated width from OSM tags
// Smoothly interpolate between street segments with different widths.
// Collect polylines between intersections only, skip the width check, and for each side (left or right outline), handle separately.

// Fuck all the above.
// This needs to be manually added by mapping outlines to collected polylines
// If there is an entry in the supplemental file, it will be used to refine the mesh of the street
// An entry consists of 2 nodes (from intersection to intersection)


template <>
struct way_network_traits<mesh_builder::highway>
{
    static way_type way_type(const mesh_builder::highway* way) {
        return way->type;
    }
};

using way_net = way_network<mesh_builder::highway>;

static std::vector<way_net::path> get_all_paths_bw_intersections(way_net& network, way_net::node_itr start_node)
{
    std::vector<way_net::path> ret;

    auto& adj_node_ids = start_node->second.adj_node_ids;
    if (adj_node_ids.size() == 2)
    {
        int index = 0;
        bool collected[2] = { false, false };
        way_net::path paths[2];

        for (auto adj_nodeid : adj_node_ids) {
            collected[index] = network.path_to_intersection(start_node, adj_nodeid, paths[index]);
            index++;
        }

        // start_node is in the middle of a path, merge both sides into one path
        if (collected[0] && collected[1] && paths[0].type == paths[1].type)
        {
            auto& path0_nodes = paths[0].nodes;
            auto& path1_nodes = paths[1].nodes;

            std::reverse(path0_nodes.begin(), path0_nodes.end());

            // reverse ways
            for (size_t i = 1; i < path0_nodes.size(); ++i) {
                path0_nodes[i].in_way = path0_nodes[i - 1].in_way;
            }
            path0_nodes[0].in_way = nullptr;
            path1_nodes[0].in_way = path0_nodes.back().in_way;
            path0_nodes.pop_back();

            for (const auto& vert : path1_nodes) {
                path0_nodes.push_back(vert);
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
            if (network.path_to_intersection(start_node, adj_nodeid, path)) {
                ret.push_back(std::move(path));
            }
        }
    }

    return ret;
}

static void gen_path_drawdata(draw_datad& dd, const way_net::path& path, double eps)
{
    // todo: when I was drawing all the ways before, it looked closer to what it actually is on google maps
    // Maybe I need to assign width based on available space/nearby footpaths
    // 
    // Don't assign width, instead use the nearby footpaths to trace the outlines of the streets
    // And draw the street texture within those outlines!
    // Some streets may not have footpaths. In that case, I can use the current strat (drawing polylines),
    // or use neighbouring buildings/relations to figure it out?
    // 

    std::vector<glm::dvec2> verts;
    verts.reserve(path.nodes.size());
    for (const auto& node : path.nodes) {
        verts.push_back(node.vert);
    }
    polyline_triangulate(verts, path.nodes[1].in_way->width, dd, eps);

    //uint32_t vert_startidx = uint32_t(dd.num_verts());
    //
    //for (const auto& vert : polyline.verts) {
    //    dd.add_vertex(vert.x, vert.y, 0);
    //}
    //for (size_t i = 0; i < polyline.verts.size() - 1; ++i) {
    //    dd.add_line(i + vert_startidx, i + 1 + vert_startidx);
    //}
}


bool mesh_builder::gen_street_drawdata(std::vector<draw_datad>& drawdata, const aabb_tree<building*>& bldg_tree)
{
    way_net network;

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

    constexpr double eps = 1e-9;

    std::vector<way_net::path> footpaths;
    std::vector<way_net::path> streets;

    // traverse through all footpaths, shoot rays to nearby streets
    // and mark those as streets as near
    // 

    // on cutting footpaths to match nearby streets:
    // go through all footpaths above. Each footpath can be cut into multiple footpath pieces
    // The cut is made when the distance or and angular tolerance to nearby street is broken
    // or more precisely, when the matched street segment changes
    // This allows cutting footpaths to match their corresponding street segments

    for (auto nodeitr = network.nodes.begin(); nodeitr != network.nodes.end(); ++nodeitr)
    {
        for (auto&& path : get_all_paths_bw_intersections(network, nodeitr))
        {
            if (path.type == WAY_TYPE_FOOTWAY) {
                footpaths.push_back(std::move(path));
            } else {
                streets.push_back(std::move(path));
            }
        }
        
        //logMESSAGE("node polylines size: %zu", node_polylines.size());

        draw_datad footpath_dd;
        footpath_dd.color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);

        for (const auto& path : footpaths) {
            gen_path_drawdata(footpath_dd, path, eps);
        }

        draw_datad street_dd;
        street_dd.color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f);

        for (const auto& street : streets) {
            gen_path_drawdata(street_dd, street, eps);
        } 

        drawdata.push_back(std::move(footpath_dd));
        drawdata.push_back(std::move(street_dd));
    }


    return true;
}
