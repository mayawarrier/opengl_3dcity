#ifndef OSM_WAY_NETWORK_HPP
#define OSM_WAY_NETWORK_HPP

#include <boost/functional/hash.hpp>

#include "../common.hpp"
#include "fwd.hpp"


struct way_net_node
{
    glm::dvec2 vert;
    types::small_flat_set<osmium::object_id_type, 4> adj_node_ids;
};

template <class TWay>
struct way_net_edge {
    const TWay* way;
};

struct way_net_edge_idpair
{
    osmium::object_id_type first;
    osmium::object_id_type second;

    struct hash {
        std::size_t operator()(const way_net_edge_idpair& s) const noexcept
        {
            std::size_t seed = 0;
            auto minmax = std::minmax(s.first, s.second);
            boost::hash_combine(seed, minmax.first);
            boost::hash_combine(seed, minmax.second);
            return seed;
        }
    };
    struct equals {
        bool operator()(const way_net_edge_idpair& lhs, const way_net_edge_idpair& rhs) const noexcept {
            return std::minmax(lhs.first, lhs.second) == std::minmax(rhs.first, rhs.second);
        }
    };
};

template <class TValue>
using way_net_edge_map_t = types::unord_flat_map<
    way_net_edge_idpair, TValue, way_net_edge_idpair::hash, way_net_edge_idpair::equals>;

using way_net_edge_set_t = types::unord_flat_set<
    way_net_edge_idpair, way_net_edge_idpair::hash, way_net_edge_idpair::equals>;


template <class GetSubTypeCb, class TWay>
concept WayNetGetSubTypeCb = requires(GetSubTypeCb cb, const TWay* way) {
    { std::invoke(cb, way) } -> std::convertible_to<int>;
};

// A network/graph of OSM ways (streets, footpaths, waterways etc.)
template <class TWay, class Traits>
    requires WayNetworkTraits<Traits, TWay>
struct way_network
{
public:
    using way_type_t = typename Traits::way_type_t;
    using way_type_flags_t = typename Traits::way_type_flags_t;

    using nodes_t = types::unord_flat_map<osmium::object_id_type, way_net_node>;
    using edges_t = way_net_edge_map_t<way_net_edge<TWay>>;

    using node_itr = typename nodes_t::iterator;
    using edge_itr = typename edges_t::iterator;
    using node_const_itr = typename nodes_t::const_iterator;
    using edge_const_itr = typename edges_t::const_iterator;

public:
    node_itr get_or_add_node(osmium::object_id_type id, glm::dvec2 vert)
    {
        auto nodeitr = m_nodes.find(id);
        if (nodeitr == m_nodes.end()) {
            way_net_node node { .vert = vert, .adj_node_ids = {} };
            nodeitr = m_nodes.insert({ id, std::move(node) }).first;
        }
        return nodeitr;
    }

    edge_itr add_edge(way_net_edge_idpair node_ids, const TWay* way)
    {
        //assert(!edges.contains(node_ids)); // todo: check why this fails

        way_net_edge<TWay> e = { .way = way };
        return m_edges.insert({ node_ids, std::move(e) }).first;
    }

    struct path
    {
        struct node 
        {
            osm_node osm_node;
            // Way into this node from the previous node,
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
        int subtype;
    };

    // Collect a path from start_node until the next intersection.
    // \param start_node - node to start from
    // \param edges_visited - map of edges already visited
    // \param path_type - type of path to get (eg. highway, waterway etc.)
    // \param get_subtype_cb - callback to get way subtype (eg. motorway, service road, etc.)
    //
    template <class Cb> requires WayNetGetSubTypeCb<Cb, TWay>
    bool get_path_to_intersection(
        node_const_itr start_node,
        way_net_edge_set_t& edges_visited,
        way_type_t path_type,
        Cb get_subtype_cb,
        path& out_path
    ) const
    {
        std::vector<typename path::node> path_nodes;
        path_nodes.push_back({
            .osm_node = osm_node(start_node->first, start_node->second.vert),
            .in_way = nullptr
        });

        osmium::object_id_type cur_nodeid = -1;
        node_const_itr next_nodeitr = start_node;
        edge_const_itr cur_edge_itr = m_edges.end();

        int prev_edge_subtype = -1;
        auto next_edge = [&]()
        {
            auto& next_node_adj_ids = next_nodeitr->second.adj_node_ids;
            assert_msg(cur_nodeid == -1 || next_node_adj_ids.contains(cur_nodeid), 
                "Node %lld should have %lld as adjacent node", next_nodeitr->first, cur_nodeid);

            // Stop at intersections (but allow starting at one).
            if (cur_nodeid != -1 && next_node_adj_ids.size() > 2) {
                return false;
            }

            int next_edge_subtype = -1;
            edge_const_itr next_edge_itr = m_edges.end();
            node_const_itr next_next_nodeitr = m_nodes.end();
            
            for (auto adj_id : next_node_adj_ids) 
            {
                if (adj_id == cur_nodeid) {
                    continue;
                }
                auto adj_id_itr = m_nodes.find(adj_id);
                if (adj_id_itr == m_nodes.end()) {
                    continue; // out of map bounds
                }
                way_net_edge_idpair edge_ids{ next_nodeitr->first, adj_id };
                if (!edges_visited.contains(edge_ids))
                {
                    auto edge_itr = m_edges.find(edge_ids);
                    assert_msg(edge_itr != m_edges.end(), 
                        "Missing edge between adjacent nodes %lld and %lld", next_nodeitr->first, adj_id);

                    if (!Traits::type_flags(edge_itr->second.way)[path_type]) {
                        continue;
                    }
                    int edge_subtype = std::invoke(get_subtype_cb, edge_itr->second.way);
                    if (prev_edge_subtype != -1 && prev_edge_subtype != edge_subtype) {
                        continue;
                    }
                    next_next_nodeitr = adj_id_itr;
                    next_edge_itr = edge_itr;
                    next_edge_subtype = edge_subtype;
                    break;
                }
            }
            if (next_edge_itr == m_edges.end()) {
                return false;
            }

            cur_nodeid = next_nodeitr->first;
            next_nodeitr = next_next_nodeitr;
            cur_edge_itr = next_edge_itr;
            prev_edge_subtype = next_edge_subtype;
            return true;
        };

        auto visit_edge = [&]()
        {
            path_nodes.push_back({
                .osm_node = osm_node(next_nodeitr->first, next_nodeitr->second.vert),
                .in_way = cur_edge_itr->second.way
            });
            edges_visited.insert(cur_edge_itr->first);
        };

        while (next_edge()) {
            visit_edge();
        }

        if (path_nodes.size() < 2) {
            return false;
        }

        assert(prev_edge_subtype != -1);       
        out_path = { 
            .nodes = std::move(path_nodes), 
            .subtype = prev_edge_subtype
        };
        return true;
    }

    // Get all paths in the graph, terminating at intersections.
    template <class GetSubTypeCb>
        requires WayNetGetSubTypeCb<GetSubTypeCb, TWay>
    std::vector<path> get_all_paths_to_intersections(way_type_t path_type, GetSubTypeCb get_subtype_cb) const
    {
        std::vector<path> ret;
        way_net_edge_set_t edges_visited;

        auto get_path = [&](node_const_itr nodeitr, path& out_path) {
            return get_path_to_intersection(nodeitr, edges_visited, path_type, get_subtype_cb, out_path);
        };

        // traverse outwards from _all_ nodes instead of just intersections to ensure
        // that disconnected loops (for eg. racetracks) are not missed
        for (auto nodeitr = m_nodes.begin(); nodeitr != m_nodes.end(); ++nodeitr)
        {
            auto& adj_node_ids = nodeitr->second.adj_node_ids;
            if (adj_node_ids.size() == 2)
            {
                path paths[2];
                bool collected[2] = { false, false };
                
                for (int i = 0; i < 2; ++i) {
                    collected[i] = get_path(nodeitr, paths[i]);
                }
                // node is in the middle of a path, merge both sides
                if (collected[0] && collected[1] && paths[0].subtype == paths[1].subtype)
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
                for (size_t i = 0; i < adj_node_ids.size(); ++i) {
                    path path;
                    if (get_path(nodeitr, path)) {
                        ret.push_back(std::move(path));
                    }
                }
            }
        }

        return ret;
    }

    const nodes_t& nodes() const { return m_nodes; }
    const edges_t& edges() const { return m_edges; }

private:
    nodes_t m_nodes;
    edges_t m_edges;
};

#endif