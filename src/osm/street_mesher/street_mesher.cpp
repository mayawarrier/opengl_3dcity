
#include "common.hpp"
#include "../mesh.hpp"

#include "st_ft_outline_builder.hpp"


bool mesh_builder::add_highway(const highway_info& info)
{
    auto& in_nodes = info.way->nodes();
    std::vector<osm_node> nodes(in_nodes.size());

    for (size_t i = 0; i < in_nodes.size(); ++i)
    {
        auto& nr = in_nodes[i];
        auto proj = osmium::geom::MercatorProjection{}(nr.location());
        nodes[i] = { nr.ref(), glm::dvec2(proj.x, proj.y) };
    }
    m_num_highway_nodes += nodes.size();

    const char* name = info.way->tags()["name"];
    m_highways.push_back({
        .id = info.way->id(),
        .name = name ? name : "",
        .type = info.type,
        .layer = info.layer,
        .nodes = std::move(nodes),
        .width = info.width,
    });

    return true;
}

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

    auto st_outline_map = build_st_ft_outlines(network, all_paths, *bldg_tree_ptr, eps);

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
        for (const auto& footpath : all_paths.footpaths) {
            gen_path_drawdata(footpath_dd, footpath, eps);
        }
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
