
#include <algorithm>
#include <span>
#include <array>
#include <unordered_map>
#include <iterator>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/pool/pool_alloc.hpp>

#include "containers/way_network.hpp"
#include "containers/aabb_tree.hpp"
#include "geom.hpp"
#include "mesh.hpp"


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

bool mesh_builder::add_highway(const highway_info& info)
{
    std::vector<osm_node> nodes;
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

using way_net = way_network<mesh_builder::highway>;

// Get all paths between intersections including this node.
// Traversing outwards from _all_ nodes instead of just intersections ensures
// that disconnected components (for eg. racetracks) are not missed.
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

        // start_node is in the middle of a path, merge both sides.
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
            if (network.path_to_intersection(start_node, adj_nodeid, path)) {
                ret.push_back(std::move(path));
            }
        }
    }

    return ret;
}

// Segment between two nodes in the network.
struct path_seg
{
    osm_node start, end;
    bbox2d bbox;
    const mesh_builder::highway* way;

    const way_net::path* path;
    int prev_segidx;
    int next_segidx;

    std::vector<int> piece_ids;

    struct aabb_traits
    {
        static const bbox2d& bbox(const path_seg* seg) {
            return seg->bbox;
        }
    };
};

static std::vector<path_seg> get_path_segments(const std::vector<way_net::path>& paths)
{
    std::vector<path_seg> ret;

    for (auto& path : paths)
    {
        assert_msg(path.nodes.size() >= 2, "bad path");

        size_t startidx = ret.size();
        for (size_t i = 0; i < path.nodes.size() - 1; ++i)
        {
            auto& start = path.nodes[i];
            auto& end = path.nodes[i + 1];

            bbox2d bbox;
            bbox.extend(start.vert);
            bbox.extend(end.vert);

            int prev_segidx = i > 0 ? int(startidx + i - 1) : -1;
            int next_segidx = i < path.nodes.size() - 2 ? int(startidx + i + 1) : -1;

            ret.push_back({
                .start = start.osm_node(),
                .end = end.osm_node(),
                .bbox = bbox,
                .way = end.in_way,
                .path = &path,
                .prev_segidx = prev_segidx,
                .next_segidx = next_segidx
            });
        }
    }

    return ret;
}

struct ray_hit
{
    double dist;
    path_seg* seg;
};

static bool ray_street_hit(const ray2d& ray, const path_seg* src_seg,
    const aabb_tree<path_seg*>& seg_tree, const aabb_tree<mesh_builder::building*>& bldg_tree,
    ray_hit& out_hit, double eps)
{
    auto seg_hit_cb = [&](const ray2d& ray, path_seg* cand_seg, 
        double& out_canddist, param_range dist_range, double eps) -> bool
    {
        if (cand_seg == src_seg) {
            return false; 
        }
        // check if footpath and street are somewhat parallel
        if (cand_seg->way->type == WAY_TYPE_STREET) 
        {
            double angle_bw = min_angle_between(ray.dir, cand_seg->end.vert - cand_seg->start.vert);
            if (std::abs(glm::degrees(angle_bw) - 90.0) > 45.0) {
                return false;
            }
        }

        segment ray_ptseg = { ray.at_point(dist_range.min), ray.at_point(dist_range.max) };
        segment cand_ptseg = { cand_seg->start.vert, cand_seg->end.vert };

        seg_inter_result inter_res;
        if (segments_intersect(ray_ptseg, cand_ptseg, inter_res, eps)) {
            out_canddist = glm::length(inter_res.point - ray.origin);
            return true;
        }
        else { return false; }
    };

    constexpr param_range dist_range{ .min = 0.0, .max = 20.0 };

    path_seg* seg_hit = nullptr;
    double seg_hitdist = std::numeric_limits<double>::infinity();
    if (seg_tree.ray_first_hit(ray, seg_hit_cb, seg_hitdist, seg_hit, dist_range, eps)) 
    {
        if (seg_hit->way->type == WAY_TYPE_STREET) {
            out_hit.dist = seg_hitdist;
            out_hit.seg = seg_hit;
            return true;
        }
    } 
    
    out_hit.dist = std::numeric_limits<double>::infinity();
    out_hit.seg = nullptr;
    return false;
};

//todo: I can start cleaning up now and reimplementing parts of the code
// some footpaths are inside area-mapped highways, so I need to check if the footpath is inside an area and remove it, just generate the area
// some footpaths are at different heights than the street! These should be ignored or drawn appropriately
// some footpaths obscure other footpaths, so I need to check if the footpath is obscured by a building or another footpath

// in order to fill in the street outlines where footpaths are missing,
// I need to determine not only which street segments are missing footpaths
// but also for long street segments, I need to break them up into pieces and determine the
// pieces outside the footpath coverage area. These pieces can then be joined, and
// polylines offset by half the street width can be drawn as the street outline in
// these areas. Note that the street extends beyond the footpath coverage area at the
// endpoints, so these will have to be handled specially.

// need to traverse the street segments, and determine which segments
// do not intersect any footpath outlines, and cut the street segments accordingly
// the last intersected footpath outline will be the outline that will be extended 
// until the end of the street. at the endpoints, need to check the adjacent streets
// and use that to determine the part that should not be filled in (intersection area)

struct outline_node
{
    // -1 if point is generated and not an OSM node.
    osmium::object_id_type id;
    glm::dvec2 vert;

    outline_node() = default;

    outline_node(const osm_node& n) : 
        id(n.id), vert(n.vert)
    {}
};

using street_outlines_t = types::unord_flat_map<const way_net::path*, std::vector<outline_node>>;

static street_outlines_t gen_street_outlines(
    const std::vector<way_net::path>& footpaths, 
    const std::vector<way_net::path>& streets, 
    const aabb_tree<mesh_builder::building*>& bldg_tree,
    double eps)
{
    auto footpath_segments = get_path_segments(footpaths);
    auto street_segments = get_path_segments(streets);

    aabb_tree<path_seg*> seg_tree;
    {
        std::vector<path_seg*> tree_objects;
        for (auto& seg : footpath_segments) {
            tree_objects.push_back(&seg);
        }
        for (auto& seg : street_segments) {
            tree_objects.push_back(&seg);
        }
        seg_tree = aabb_tree<path_seg*>::create_unsafe(tree_objects);
    }

    struct outline_piece
    {
        outline_node start, end;
        const path_seg* seg;
        bool joined = false;
    };

    std::vector<outline_piece> pieces;
    types::unord_flat_map<const way_net::path*, types::flat_set<int>> street_piece_ids;

    types::unsync_pool_alloc<ray_hit> rayhit_alloc;
    types::unsync_pool_alloc<glm::dvec2> rayorig_alloc;

    auto add_outline_pieces = [&](path_seg& seg, std::span<const glm::dvec2> ray_origins, glm::dvec2 ray_dir)
    {
        size_t num_rays = ray_origins.size();
        auto ray_hits = rayhit_alloc.allocate(num_rays);

        for (size_t i = 0; i < num_rays; ++i)
        {
            ray2d ray{ .origin = ray_origins[i], .dir = ray_dir };
            ray_street_hit(ray, &seg, seg_tree, bldg_tree, ray_hits[i], eps);
        }

        size_t rayidx = 0;
        while (rayidx < num_rays)
        {
            // get largest piece that hits the same street
            size_t pc_startidx = rayidx;
            while (pc_startidx < num_rays && !ray_hits[pc_startidx].seg) {
                pc_startidx++;
            }
            size_t pc_endidx = pc_startidx + 1;
            while (pc_endidx < num_rays && ray_hits[pc_endidx].seg &&
                ray_hits[pc_endidx].seg->path == ray_hits[pc_startidx].seg->path) {
                pc_endidx++;
            }

            if (pc_startidx < num_rays)
            {
                outline_piece piece;
                piece.seg = &seg;

                if (pc_startidx == 0) {
                    piece.start = seg.start;
                } else {
                    piece.start.id = -1;
                    piece.start.vert = segment_idx_mid(ray_origins, pc_startidx - 1, pc_startidx);
                }
                if (pc_endidx == num_rays) {
                    piece.end = seg.end;
                } else {
                    piece.end.id = -1;
                    piece.end.vert = segment_idx_mid(ray_origins, pc_endidx - 1, pc_endidx);
                }

                pieces.push_back(piece);

                int piece_id = int(pieces.size() - 1);
                const auto* hit_street = ray_hits[pc_startidx].seg->path;

                seg.piece_ids.push_back(piece_id);
                street_piece_ids[hit_street].insert(piece_id);
            }

            rayidx = pc_endidx;
        }

        rayhit_alloc.deallocate(ray_hits, num_rays);
    };

    static constexpr double RAY_FIRE_INTERVAL = 5.0; // meters

    for (auto& fseg : footpath_segments)
    {
        glm::dvec2 seg_vec = fseg.end.vert - fseg.start.vert;
        glm::dvec2 seg_perp_dir = glm::normalize(vec_perp(seg_vec));

        double seg_length = glm::length(seg_vec);
        int num_rays = std::max(1, int(std::ceil(seg_length / RAY_FIRE_INTERVAL - eps))) + 1;

        auto ray_origins = rayorig_alloc.allocate(num_rays);
        for (int i = 0; i < num_rays; ++i) {
            ray_origins[i] = fseg.start.vert + (double(i) / num_rays) * seg_vec;
        }

        add_outline_pieces(fseg, { ray_origins, size_t(num_rays) }, seg_perp_dir);
        add_outline_pieces(fseg, { ray_origins, size_t(num_rays) }, -seg_perp_dir);

        rayorig_alloc.deallocate(ray_origins, num_rays);
    }

    street_outlines_t ret;
    for (auto& [street, st_piece_ids] : street_piece_ids)
    {
        boost::container::small_vector<std::vector<outline_node>, 5> outlines;

        for (auto st_pid : st_piece_ids)
        {
            auto* start_piece = &pieces[st_pid];
            if (start_piece->joined) {
                continue;
            }

            auto& outline = outlines.emplace_back();

            // go to the start of the piece chain
            auto* cur_piece = start_piece;
            while (true)
            {
                int prev_segidx = cur_piece->seg->prev_segidx;
                auto* prev_seg = prev_segidx != -1 ? &footpath_segments[prev_segidx] : nullptr;

                if (cur_piece->start.id == -1 || !prev_seg || prev_seg->piece_ids.empty()) {
                    break;
                }
                int prev_piece_id = prev_seg->piece_ids.back();
                if (!st_piece_ids.contains(prev_piece_id)) {
                    break;
                }
                auto* prev_piece = &pieces[prev_piece_id];
                if (prev_piece->end.id != cur_piece->start.id) {
                    break;
                }

                cur_piece = prev_piece;
            }

            while (cur_piece != start_piece)
            {
                cur_piece->joined = true;
                outline.push_back(cur_piece->start);

                auto& next_seg = footpath_segments[cur_piece->seg->next_segidx];
                cur_piece = &pieces[next_seg.piece_ids.front()];

                // every middle piece should be a complete segment
                assert(cur_piece == start_piece || 
                    (cur_piece->seg->piece_ids.size() == 1 && 
                        cur_piece->start.id != -1 && cur_piece->end.id != -1));
            }

            // extend chain forwards
            while (true)
            {
                cur_piece->joined = true;
                outline.push_back(cur_piece->start);

                int next_segidx = cur_piece->seg->next_segidx;
                auto* next_seg = next_segidx != -1 ? &footpath_segments[next_segidx] : nullptr;

                if (cur_piece->end.id == -1 || !next_seg || next_seg->piece_ids.empty()) {
                    break;
                }
                int next_piece_id = next_seg->piece_ids.front();
                if (!st_piece_ids.contains(next_piece_id)) {
                    break;
                }
                auto* next_piece = &pieces[next_piece_id];
                if (next_piece->start.id != cur_piece->end.id) {
                    break;
                }

                cur_piece = next_piece;
            }

            outline.push_back(cur_piece->end);
        }

        assert_msg(!outlines.empty(), "No pieces but has outline entry?");
        assert_msg(std::in_range<int>(outlines.size()), "num outlines %z is absurd", outlines.size());

        // this does not scale well (O(n^3) and a lot of copying/reversing), but
        // it should be okay as the number of outlines are usually small (2-3).
        while (outlines.size() > 1)
        {
            int best_i = -1, best_j = -1;
            bool reverse_i = false, reverse_j = false;
            double min_dist = std::numeric_limits<double>::max();
        
            for (int i = 0; i < int(outlines.size()); ++i) {
                for (int j = 0; j < int(outlines.size()); ++j)
                {
                    if (i == j) { continue; }

                    // check all possible pairs of endpoints
                    std::array<double, 4> dists = {
                        vec_sqlength(outlines[i].back().vert - outlines[j].front().vert),
                        vec_sqlength(outlines[i].back().vert - outlines[j].back().vert),
                        vec_sqlength(outlines[i].front().vert - outlines[j].front().vert),
                        vec_sqlength(outlines[i].front().vert - outlines[j].back().vert),
                    };

                    int min_distidx = int(std::min_element(dists.begin(), dists.end()) - dists.begin());
                    if (dists[min_distidx] < min_dist) 
                    {
                        min_dist = dists[min_distidx];
                        reverse_i = get_bit(min_distidx, 1); // nasty
                        reverse_j = get_bit(min_distidx, 0);
                        best_i = i; best_j = j;
                    }
                }
            }

            auto& outlineI = outlines[best_i];
            auto& outlineJ = outlines[best_j];
        
            if (reverse_i) { std::reverse(outlineI.begin(), outlineI.end()); }
            if (reverse_j) { std::reverse(outlineJ.begin(), outlineJ.end()); }
        
            outlineI.insert(outlineI.end(), outlineJ.begin(), outlineJ.end());
            outlines.erase(outlines.begin() + best_j);
        }

        if (outlines[0].size() > 2) {
            ret[street] = std::move(outlines[0]);
        }
        //else assert(false); // todo: check why this fails
    }

    return ret;
}

static void gen_path_drawdata(draw_datad& dd, const way_net::path& path, double eps)
{
    std::vector<glm::dvec2> verts;
    verts.reserve(path.nodes.size());
    for (const auto& node : path.nodes) {
        verts.push_back(node.vert);
    }
    // todo: use each way's width when triangulating the polyline
    polyline_triangulate(verts, path.nodes[1].in_way->width, dd, eps);
}

bool mesh_builder::gen_street_drawdata(std::vector<draw_datad>& drawdata, const aabb_tree<building*>* bldg_tree_ptr)
{
    auto tbegin = clk::now();

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

    std::vector<way_net::path> footpaths, streets;
    for (auto nodeitr = network.nodes.begin(); nodeitr != network.nodes.end(); ++nodeitr)
    {
        auto node_paths = get_all_paths_bw_intersections(network, nodeitr);
        for (auto& path : node_paths) 
        {
            if (path.type == WAY_TYPE_FOOTWAY) {
                footpaths.push_back(std::move(path));
            }
            else if (path.type == WAY_TYPE_STREET) {
                streets.push_back(std::move(path));
            }
        }
    }

    auto& bldg_tree = *bldg_tree_ptr;
    auto street_outlines_map = gen_street_outlines(footpaths, streets, bldg_tree, eps);

    draw_datad footpath_dd{ 
        .name = "footpaths",
        .color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f)
    };
    draw_datad street_dd{ 
        .name = "streets",
        .color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f)
    };

    draw_datad debug_dd{
        .name = "debug",
        .color = glm::vec4(1.0f, 0.f, 0.f, 1.f)
    };

    // for each footpath, generate drawdata
    for (const auto& footpath : footpaths)
    {
        gen_path_drawdata(footpath_dd, footpath, eps);
    }

    //std::pmr::polymorphic_allocator<glm::dvec2> vert_alloc(&mempool);
    types::unsync_pool_alloc<glm::dvec2> vert_alloc;

    for (auto& [street, outline] : street_outlines_map)
    {
        //if (std::find_if(outline_itr->first->nodes.begin(), outline_itr->first->nodes.end(),
        //    [](auto& n) { return n.id == 1801247228; }) != outline_itr->first->nodes.end())
        //{
        //    logMESSAGE("Found street node with id 1801247228");
        //}

        auto* outline_verts_ptr = vert_alloc.allocate(outline.size());
        //auto outline_verts_ptr = std::make_unique<glm::dvec2[]>(outline.size());

        std::span outline_verts(outline_verts_ptr, outline.size());
        for (size_t i = 0; i < outline.size(); ++i) {
            outline_verts[i] = outline[i].vert;
        }
        
        orient_t outline_orient = polygon_orient(outline_verts);
        if (outline_orient == ORIENT_COLL) {
            logMESSAGE("Skipping outline as it has 0 area");
            //assert(false); // todo: figure out why these are failing
            vert_alloc.deallocate(outline_verts_ptr, outline.size());
            continue;
        }

        auto tri_indices = polygon_triangulate(outline_verts, outline_orient);
        if (tri_indices.empty()) {
            vert_alloc.deallocate(outline_verts_ptr, outline.size());
            continue;
        }
        
        // 763664859      - 7
        // 11070333726    - 6
        // 394499209      - 2

        //if (std::find_if(outline_itr->first->nodes.begin(), outline_itr->first->nodes.end(),
        //    [](auto& n) { return n.id == 931377041; }) != outline_itr->first->nodes.end() &&
        //    std::find_if(outline_itr->first->nodes.begin(), outline_itr->first->nodes.end(),
        //        [](auto& n) { return n.id == 11070333727; }) != outline_itr->first->nodes.end())
        //{
        //    debug_dd.add_vertex({ joined_outline[8].second, 0.0 });
        //    debug_dd.add_vertex({ joined_outline[3].second, 0.0 });
        //    debug_dd.add_vertex({ joined_outline[0].second, 0.0 });
        //    debug_dd.add_triangle(0, 1, 2);
        //}
        //else {
        uint32_t vert_startidx = uint32_t(street_dd.num_verts());
        for (const auto& point : outline_verts) {
            street_dd.add_vertex(point.x, point.y, 0.0);
        }
        for (size_t i = 0; i < tri_indices.size(); i += 3) {
            street_dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
        }

        vert_alloc.deallocate(outline_verts_ptr, outline.size());
        //}
    }

    for (auto & street : streets)
    {
        //if (!street_outlines_map.contains(&street)) {
            gen_path_drawdata(street_dd, street, eps);
        //}
        
    }

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Street gen took: %s", time_str(tend - tbegin).c_str());

    return true;
}
