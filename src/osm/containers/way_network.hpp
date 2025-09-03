#ifndef OSM_WAY_NETWORK_HPP
#define OSM_WAY_NETWORK_HPP

#ifdef NDEBUG
#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/flat_set.hpp>
#else
#include <unordered_map>
#include <set>
#endif

#include <boost/functional/hash.hpp>
#include <glm/glm.hpp>
#include <osmium/osm/types.hpp>

#include "../common.hpp"


template <typename TWay>
struct way_network_traits
{
    static_assert(deferred_false<TWay>::value,
        "way_network_traits must be specialized for the TWay type.");

    static way_type way_type(const TWay* way) {
        (void)way;
        return WAY_TYPE_UNKNOWN;
    }
};

// A network/graph of OSM ways (streets, footpaths, etc.)
template <typename TWay>
struct way_network
{
    using traits = way_network_traits<TWay>;

    struct node
    {
        glm::dvec2 vert;
        // Edge duplication is possible if two ways share segments, so use a set.
        // Two edges can also have the same way if a way loops back to 
        // its starting node.
        types::flat_set<osmium::object_id_type> adj_node_ids;
    };

    struct edge
    {
        const TWay* way;
        bool visited;
    };

    using edge_idpair = std::pair<osmium::object_id_type, osmium::object_id_type>;

    // order-invariant
    struct edge_idpair_hash
    {
        std::size_t operator()(const edge_idpair& s) const noexcept
        {
            std::size_t seed = 0;
            auto minmax = std::minmax(s.first, s.second);
            boost::hash_combine(seed, minmax.first);
            boost::hash_combine(seed, minmax.second);
            return seed;
        }
    };
    // order-invariant
    struct edge_idpair_equals
    {
        bool operator()(const edge_idpair& lhs, const edge_idpair& rhs) const noexcept {
            return std::minmax(lhs.first, lhs.second) == std::minmax(rhs.first, rhs.second);
        }
    };

    types::unord_flat_map<osmium::object_id_type, node> nodes;
    types::unord_flat_map<edge_idpair, edge, edge_idpair_hash, edge_idpair_equals> edges;

    using node_itr = decltype(nodes)::iterator;
    using edge_itr = decltype(edges)::iterator;

    node_itr get_or_add_node(osmium::object_id_type id, glm::dvec2 vert)
    {
        auto nodeitr = nodes.find(id);
        if (nodeitr == nodes.end())
        {
            node node = { .vert = vert, .adj_node_ids = {} };
            nodeitr = nodes.insert({ id, std::move(node) }).first;
        }
        return nodeitr;
    }

    edge_itr add_edge(edge_idpair node_ids, const TWay* way)
    {
        //assert(!edges.contains(node_ids)); // todo: check why this fails

        edge e = { .way = way, .visited = false };
        return edges.insert({ node_ids, std::move(e) }).first;
    }

    struct path
    {
        struct node 
        {
            osmium::object_id_type id;
            glm::dvec2 vert;
            // way into this node from the previous node,
            // null for the first node in the path
            const TWay* in_way;

            osm_node osm_node() const noexcept { 
                return { id, vert }; 
            }
        };
        std::vector<node> nodes;
        way_type type;
    };

    // Collect path to the nearest intersection.
    // Marks edges as visited along the way.
    // \param nodeitr - iterator to starting node
    // \param adj_nodeid - id of adjacent node to start collecting from
    bool path_to_intersection(node_itr start_nodeitr, osmium::object_id_type adj_nodeid, path& out_path)
    {
        assert(start_nodeitr->second.adj_node_ids.contains(adj_nodeid));

        std::vector<path::node> path_nodes;

        auto cur_nodeid = start_nodeitr->first;
        auto next_nodeid = adj_nodeid;
        const TWay* prev_edgeway = nullptr;

        path_nodes.push_back({
            .id = start_nodeitr->first,
            .vert = start_nodeitr->second.vert,
            .in_way = nullptr
        });

        while (true)
        {
            node_itr next_nodeitr = nodes.find(next_nodeid);
            if (next_nodeitr == nodes.end()) {
                break; // out of map bounds
            }

            edge_itr edgeitr = edges.find({ cur_nodeid, next_nodeid });
            assert_msg(edgeitr != edges.end(),
                "Missing graph edge between nodes %lld and %lld", cur_nodeid, next_nodeid);

            auto& cur_edge = edgeitr->second;

            bool can_visit_edge = false;
            if (!cur_edge.visited) 
            {
                if (!prev_edgeway) {
                    can_visit_edge = true;
                }
                else {
                    way_type prev_type = traits::way_type(prev_edgeway);
                    way_type cur_type = traits::way_type(cur_edge.way);
                    assert(prev_type != WAY_TYPE_UNKNOWN && cur_type != WAY_TYPE_UNKNOWN);

                    can_visit_edge = (prev_type == cur_type);
                }
            }
            if (!can_visit_edge) {
                break;
            }

            path_nodes.push_back({
                .id = next_nodeitr->first,
                .vert = next_nodeitr->second.vert,
                .in_way = cur_edge.way
            });

            cur_edge.visited = true;
            prev_edgeway = cur_edge.way;

            auto& next_node_adj_ids = next_nodeitr->second.adj_node_ids;
            assert_msg(next_node_adj_ids.size() != 0,
                "Adjacent node %lld has no adjacent nodes?", next_nodeid);

            if (next_node_adj_ids.size() > 2) {
                break; // stop at intersections
            }

            osmium::object_id_type next_next_nodeid = -1; // out of bounds id
            if (next_node_adj_ids.size() == 2) 
            {
                next_next_nodeid = *std::find_if(
                    next_node_adj_ids.begin(), next_node_adj_ids.end(), 
                    [&](auto id) { return id != cur_nodeid; });
            }

            cur_nodeid = next_nodeid;
            next_nodeid = next_next_nodeid;
        }

        if (path_nodes.size() < 2) {
            return false;
        }

        assert(prev_edgeway);       
        out_path = { 
            .nodes = std::move(path_nodes), 
            .type = traits::way_type(prev_edgeway)
        };
        return true;
    }
};

#endif