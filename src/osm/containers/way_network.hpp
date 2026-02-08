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
#include "fwd.hpp"


// A network/graph of OSM ways (streets, footpaths, etc.)
template <typename TWay, typename Traits>
struct way_network
{
public:
    static_assert(requires {
        { Traits::way_type(std::declval<const TWay*>()) } -> std::same_as<way_type>;
    }, "Invalid Traits type.");

    struct node
    {
        glm::dvec2 vert;
        // Edge duplication is possible if two ways share segments, so use a set.
        // Two edges can also have the same way if a way loops back to 
        // its starting node.
        types::flat_set<osmium::object_id_type> adj_node_ids;
    };

    struct edge {
        const TWay* way;
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

    template <class T>
    using edge_map_t = types::unord_flat_map<edge_idpair, T, edge_idpair_hash, edge_idpair_equals>;

    using nodes_t = types::unord_flat_map<osmium::object_id_type, node>;
    using edges_t = edge_map_t<edge>;
    using node_itr = typename nodes_t::iterator;
    using edge_itr = typename edges_t::iterator;

public:
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

        edge e = { .way = way };
        return edges.insert({ node_ids, std::move(e) }).first;
    }

    struct path
    {
        struct node 
        {
            osm_node osm_node;
            // way into this node from the previous node,
            // null for the first node in the path
            const TWay* in_way;

            osmium::object_id_type id() const {
                return osm_node.id; 
            }
            const glm::dvec2& vert() const { 
                return osm_node.vert; 
            }
        };
        std::vector<node> nodes;
        way_type type;
    };

    //
    // Collect path from start_node towards adj_nodeid until the next intersection.
    // \param nodeitr - iterator to starting node
    // \param adj_nodeid - id of adjacent node to start collecting from
    // \param edges_visited - map of edges already visited
    //
    bool path_to_intersection(node_itr start_node, osmium::object_id_type adj_nodeid, edge_map_t<bool>& edges_visited, path& out_path)
    {
        assert(start_node->second.adj_node_ids.contains(adj_nodeid));

        std::vector<typename path::node> path_nodes;

        auto cur_nodeid = start_node->first;
        auto next_nodeid = adj_nodeid;
        const TWay* prev_edgeway = nullptr;

        path_nodes.push_back({
            .osm_node = {
                .id = start_node->first,
                .vert = start_node->second.vert
            },
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

            bool can_visit_edge = false;
            if (!edges_visited[edgeitr->first])
            {
                if (!prev_edgeway) {
                    can_visit_edge = true;
                }
                else {
                    way_type prev_etype = Traits::way_type(prev_edgeway);
                    way_type cur_etype = Traits::way_type(edgeitr->second.way);
                    assert(prev_etype != WAY_TYPE_UNKNOWN && cur_etype != WAY_TYPE_UNKNOWN);

                    can_visit_edge = (prev_etype == cur_etype);
                }
            }
            // type does not match or already visited
            if (!can_visit_edge) {
                break;
            }

            path_nodes.push_back({
                .osm_node = {
                    .id = next_nodeitr->first,
                    .vert = next_nodeitr->second.vert,
                },
                .in_way = edgeitr->second.way
            });

            edges_visited[edgeitr->first] = true;
            prev_edgeway = edgeitr->second.way;

            // stop at intersections
            if (next_nodeitr->second.adj_node_ids.size() > 2) {
                break;
            }

            auto& next_node_adj_ids = next_nodeitr->second.adj_node_ids;
            assert_msg(next_node_adj_ids.size() != 0,
                "Adjacent node %lld has no adjacent nodes?", next_nodeid);

            osmium::object_id_type next_next_nodeid = -1;
            if (next_node_adj_ids.size() == 2) 
            {
                auto& c = next_node_adj_ids;
                next_next_nodeid = *std::find_if(c.begin(), c.end(), [&](auto id) { 
                    return id != cur_nodeid; 
                });
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
            .type = Traits::way_type(prev_edgeway)
        };
        return true;
    }

public:
    nodes_t nodes;
    edges_t edges;
};

#endif