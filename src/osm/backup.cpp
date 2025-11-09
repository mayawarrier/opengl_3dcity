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

#include "containers/way_network.hpp"
#include "containers/aabb_tree.hpp"
#include "geom.hpp"
#include "mesh.hpp"

// Strategy:
// Snap the street segments to the footpaths, if they are within a certain distance and angular tolerance
// Consider left and right outlines of the street separately.
// If footpath is not present, just draw the outline normally, by using estimated width from OSM tags.
// Smoothly interpolate between street segments with different widths.

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

static inline int path_num_segs(const way_net::path* path) {
    return int(path->nodes.size()) - 1;
}

static inline segment path_seg(const way_net::path* path, int idx) {
    return { path->nodes[idx].vert, path->nodes[idx + 1].vert };
}

static inline double path_seg_width(const way_net::path* path, int idx) {
    // +1 because in_way is null for the first node
    return path->nodes[idx + 1].in_way->width;
}

// Get direction of point relative to path segment.
static direction path_seg_pt_rel_dir(const way_net::path* path, int idx, const glm::dvec2& point)
{
    auto seg = path_seg(path, idx);
    auto pto = orient(seg.first, seg.second, point);
    switch (pto)
    {
        case ORIENT_CCW:
            return DIR_LEFT;
        case ORIENT_COLL:
            return DIR_UNDEF;
        case ORIENT_CW:
            return DIR_RIGHT;
        default:
            assert(false);
            return DIR_UNDEF;
    }
}

struct way_net_paths
{
    std::vector<way_net::path> footpaths;
    std::vector<way_net::path> streets;
    // add more as needed
};

static way_net_paths get_all_paths_bw_intersections(way_net& network)
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
    
    // Traversing outwards from _all_ nodes instead of just intersections ensures
    // that disconnected components (for eg. racetracks) are not missed.
    for (auto nodeitr = network.nodes.begin(); nodeitr != network.nodes.end(); ++nodeitr)
    {
        auto& adj_node_ids = nodeitr->second.adj_node_ids;
        if (adj_node_ids.size() == 2)
        {
            int index = 0;
            bool collected[2] = { false, false };
            way_net::path paths[2];

            for (auto adj_nodeid : adj_node_ids) {
                collected[index] = network.path_to_intersection(nodeitr, adj_nodeid, paths[index]);
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
                if (network.path_to_intersection(nodeitr, adj_nodeid, path)) {
                    add_path(std::move(path));
                }
            }
        }
    }

    return ret;
}

//todo: I can start cleaning up now and reimplementing parts of the code
// some footpaths are inside area-mapped highways, so I need to check if the footpath is inside an area and remove it, just generate the area
// some footpaths are at different heights than the street! These should be ignored or drawn appropriately
// some footpaths obscure other footpaths, so I need to check if the footpath is obscured by a building or another footpath

// need to traverse the street segments, and determine which segments
// do not intersect any footpath outlines, and cut the street segments accordingly
// the last intersected footpath outline will be the outline that will be extended 
// until the end of the street. at the endpoints, need to check the adjacent streets
// and use that to determine the part that should not be filled in (intersection area)

// Instead of firing rays, can I find the projection of the footpath outline endpoints
// on the street? This should give me the exact point where the street is no longer
// covered by the footpath outlines. But how do I know which street segment to project on? Just try
// covered by the footpath outlines. But how do I know which street segment to projection on? Just try
// the last few segments on either side? And I need an algorithm to perform projection on segments,
// not lines. Perpendicular projection? Unfortunately there might not be a perpendicular projection
// because the footpath segments could be at various angles, especially at endpoints.
// But I already know the angle between the footpath segment and the street seg it hit right?
// (in ray_hit_footpath2street)
// Could I not project the endpoint somewhere there?
// No need for the angle! I know the point on the street segment hit by the ray!
// I also know the exact segment that was hit, and its position index in the path
// But this point may not hit the street at 90 degrees. does this matter?
// assume this does not matter. Now what?
// Store the hit seg for every node in the outline. the hit seg will always belong to the same street path
// After the outlines are joint, I now know the intersected segment on both sides of the outline
// I can now extend the outline on both sides by iterating through the street segments on both sides and 
// generating an outline at an offset from the main street way. 
// 
// Which direction to iterate in???
// Direction of the street segment can be opposite to the footpath segment!
// now what?
// 

// need to know which side to extend from.


class street_outline_builder
{
public:
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

public:
    street_outline_builder(const way_net_paths& all_paths, const aabb_tree2d<mesh_builder::building*>& bldg_tree) :
        m_bldg_tree(bldg_tree)
    {
        m_footpath_segs = get_all_path_segments(all_paths.footpaths);
        m_street_segs = get_all_path_segments(all_paths.streets);

        std::vector<tree_path_seg*> tree_objects;
        for (auto& seg : m_footpath_segs) {
            tree_objects.push_back(&seg);
        }
        for (auto& seg : m_street_segs) {
            tree_objects.push_back(&seg);
        }
        m_seg_tree = aabb_tree2d<tree_path_seg*>::create_unsafe(tree_objects);
        m_seg_tree = aabb_tree2d<tree_path_seg*>::create_unsafe(std::span(tree_objects));
    }

    street_outlines_t get_outlines(double eps)
    {
        tim::high_resolution_clock::duration ttotal_outlines(0);

        auto tbegin_entries = std::chrono::high_resolution_clock::now();
        auto entries = get_all_st_ft_outlines_from_streets(eps);
        auto tend_entries = std::chrono::high_resolution_clock::now();

        logMESSAGE("Street outline extraction time: %s", time_str(tend_entries - tbegin_entries).c_str());

        street_outlines_t ret;
        for (auto& st_entry : entries)
        {
            auto& street = st_entry.street;
            auto& outlines = st_entry.outlines;

            assert(!outlines.empty());

            std::vector<const st_outline*> left_outlines, right_outlines;
            for (auto& outline : outlines)
            {
                switch (outline.dir)
                {
                case DIR_LEFT:  left_outlines.push_back(&outline); break;
                case DIR_RIGHT: right_outlines.push_back(&outline); break;
                default:
                    assert(false); break;
                }
            }

            /*if (std::find_if(street->nodes.begin(), street->nodes.end(),
                [](auto& n) { return n.id == 2155391486; }) != street->nodes.end()) {
                logMESSAGE("here");
            }*/

            auto tbegin = std::chrono::high_resolution_clock::now();
            auto lt_outline = fill_st_outline_holes(street, left_outlines, DIR_LEFT, eps);
            auto rt_outline = fill_st_outline_holes(street, right_outlines, DIR_RIGHT, eps);
            auto tend = std::chrono::high_resolution_clock::now();
            ttotal_outlines += (tend - tbegin);

            if (lt_outline.empty() || rt_outline.empty()) {
                continue;
            }

            std::array<double, 4> dists = {
                vec_sqlength(lt_outline.back().vert - rt_outline.front().vert),
                vec_sqlength(lt_outline.back().vert - rt_outline.back().vert),
                vec_sqlength(lt_outline.front().vert - rt_outline.front().vert),
                vec_sqlength(lt_outline.front().vert - rt_outline.back().vert),
            };

            int min_distidx = int(std::min_element(dists.begin(), dists.end()) - dists.begin());
            bool reverse_lt = get_bit(min_distidx, 1); // nasty
            bool reverse_rt = get_bit(min_distidx, 0);

            if (reverse_lt) { std::reverse(lt_outline.begin(), lt_outline.end()); }
            if (reverse_rt) { std::reverse(rt_outline.begin(), rt_outline.end()); }

            lt_outline.insert(lt_outline.end(), rt_outline.begin(), rt_outline.end());
            ret[street] = std::move(lt_outline);
        }

        logMESSAGE("Street outline hole filling time: %s", time_str(ttotal_outlines).c_str());

        return ret;
    }

private:
    struct tree_path_seg
    {
        osm_node start, end;
        double length;
        glm::dvec2 udir;
        bbox2d bbox;

        const mesh_builder::highway* way;
        const way_net::path* path;

        // Index in the global path segments vec
        int prev_seg_gidx, next_seg_gidx;
        // Index in the path
        int pathidx;

        struct aabb_traits {
            static const bbox2d& bbox(const tree_path_seg* seg) {
                return seg->bbox;
            }
        };
    };

    std::vector<tree_path_seg> get_all_path_segments(const std::vector<way_net::path>& paths)
    {
        std::vector<tree_path_seg> ret;

        for (auto& path : paths)
        {
            assert_msg(path.nodes.size() >= 2, "bad path");

            int startidx = int(ret.size());
            for (int i = 0; i < path_num_segs(&path); ++i)
            {
                auto& start = path.nodes[i];
                auto& end = path.nodes[i + 1];

                bbox2d bbox;
                bbox.extend(start.vert);
                bbox.extend(end.vert);

                int prev_seg_gidx = i > 0 ? (startidx + i - 1) : -1;
                int next_seg_gidx = i < (int(path.nodes.size()) - 2) ? (startidx + i + 1) : -1;

                double length = glm::length(end.vert - start.vert);

                ret.push_back({
                    .start = start.osm_node(),
                    .end = end.osm_node(),
                    .length = length,
                    .udir = (end.vert - start.vert) / length,
                    .bbox = bbox,
                    .way = end.in_way,
                    .path = &path,
                    .prev_seg_gidx = prev_seg_gidx,
                    .next_seg_gidx = next_seg_gidx,
                    .pathidx = i
                });
            }
        }

        return ret;
    }

    struct outline_proj
    {
        const way_net::path* street;
        struct data {
            double pt_param;
            int seg_pidx;
        } proj;

        static constexpr outline_proj no_proj() {
            return {
                .street = nullptr,
                .proj = { .pt_param = -1.0, .seg_pidx = -1 }
            };
        }
    };

    // ray.dir must be normalized!
    outline_proj get_street_proj(const glm::dvec2& sample, const tree_path_seg* src_tseg, double eps)
    {
        struct tree_qdata
        {
            double sqdist;
            double pt_param;
        };

        auto dist_range = param_range{ 0.0, 20.0 };

        auto cand_intersect_cb = [&](tree_path_seg* cand_tseg, tree_qdata& out_qdata) -> bool
        {
            if (cand_tseg->path == src_tseg->path) {
                return false;
            }

            // check if footpath and street are somewhat parallel
            //if (cand_tseg->way->type == WAY_TYPE_STREET)
            //{
            double angle_bw = min_angle_bw_unitdirs(src_tseg->udir, cand_tseg->udir);
            if (std::abs(glm::degrees(angle_bw)) > 45.0) {
                return false;
            }
            //}

            segment seg = { cand_tseg->start.vert, cand_tseg->end.vert };

            // project sample onto segment
            glm::dvec2 ap = sample - seg.first;
            glm::dvec2 ab = seg.second - seg.first;
            double t = glm::dot(ap, ab) / glm::dot(ab, ab);

            // reject if out of path bounds
            int seg_pidx = cand_tseg->pathidx;
            if ((t < 0 && seg_pidx == 0) ||
                (t > 1.0 && seg_pidx == path_num_segs(cand_tseg->path) - 1)) {
                return false;
            }

            double t_clamped = std::clamp(t, 0.0, 1.0);
            double sqdist = vec_sqlength(sample - (seg.first + t_clamped * ab));

            if (sqdist < dist_range.min2() || sqdist > dist_range.max2()) {
                return false;
            }

            out_qdata = {
                .sqdist = sqdist,
                .pt_param = t // keep original
            };
            return true;
        };

        tree_qdata hit_segdata{};
        tree_path_seg* hit_seg = nullptr;
        bool hit = m_seg_tree.query_nearest(sample, dist_range, cand_intersect_cb, hit_seg, hit_segdata);

        if (hit && hit_seg->way->type == WAY_TYPE_STREET) 
        {
            return {
                .street = hit_seg->path,
                .proj = {
                    .pt_param = hit_segdata.pt_param,
                    .seg_pidx = hit_seg->pathidx,
                }
            };
        }
    
        return outline_proj::no_proj();
    };

    struct st_outline
    {
        std::vector<outline_node> nodes;
        outline_proj::data start_proj, end_proj;
        direction dir; // wrt to street
    };

    struct st_outlines_entry
    {
        const way_net::path* street;
        types::small_vector<st_outline, 4> outlines;
    };

    // get street outlines from nearby footpaths
    std::vector<st_outlines_entry> get_all_st_ft_outlines(double eps)
    {
        static constexpr double SAMPLE_INTERVAL = 1; // meters

        struct outline_piece
        {
            outline_node start, end;
            outline_proj::data start_proj, end_proj;
            const tree_path_seg* seg;
            bool joined = false;
        };

        std::vector<outline_piece> pieces;
        types::unord_flat_map<const tree_path_seg*, types::small_vector<int, 2>> fseg_piece_ids;
        types::unord_flat_map<const way_net::path*, types::flat_set<int>> street_piece_ids;

        std::vector<outline_proj> projs;
        std::vector<glm::dvec2> samples;

        for (auto& fseg : m_footpath_segs)
        {
            glm::dvec2 fseg_vec = fseg.end.vert - fseg.start.vert;
            glm::dvec2 fseg_perp_dir = glm::normalize(vec_perp(fseg_vec));

            double seg_length = glm::length(fseg_vec);
            int num_samples = std::max(1, int(std::ceil(seg_length / SAMPLE_INTERVAL - eps))) + 1;

            for (int i = 0; i < num_samples; ++i) {
                samples.push_back(fseg.start.vert + (double(i) / num_samples) * fseg_vec);
            }

            //for (glm::dvec2 ray_dir : { fseg_perp_dir, -fseg_perp_dir })
            //{
                if (fseg.way->id == 366054816) {
                    logMESSAGE("debug");
                }

                for (int i = 0; i < num_samples; ++i) {
                    projs.push_back(get_street_proj(samples[i], &fseg, eps));
                }

                if (fseg.way->id == 366054816) {
                    logMESSAGE("debug");
                }

                int rayidx = 0;
                while (rayidx < num_samples)
                {
                    // get largest piece that hits the same street
                    int pc_startidx = rayidx;
                    while (pc_startidx < num_samples && !projs[pc_startidx].street) {
                        pc_startidx++;
                    }
                    int pc_endidx = pc_startidx + 1;
                    while (pc_endidx < num_samples && projs[pc_endidx].street &&
                        projs[pc_endidx].street == projs[pc_startidx].street) {
                        pc_endidx++;
                    }

                    if (pc_startidx < num_samples && pc_endidx - pc_startidx > 1)
                    {
                        outline_piece piece;
                        piece.seg = &fseg;

                        if (pc_startidx == 0) {
                            piece.start = outline_node::osm(fseg.start);
                        } else {
                            piece.start.id = -1;
                            piece.start.vert = samples[pc_startidx];
                        }
                        if (pc_endidx == num_samples) {
                            piece.end = outline_node::osm(fseg.end);
                        } else {
                            piece.end.id = -1;
                            piece.end.vert = samples[pc_endidx - 1];
                        }

                        piece.start_proj = projs[pc_startidx].proj;
                        piece.end_proj = projs[pc_endidx - 1].proj;

                        int piece_id = int(pieces.size());
                        auto* hit_st = projs[pc_startidx].street;

                        pieces.push_back(piece);
                        fseg_piece_ids[&fseg].push_back(piece_id);
                        street_piece_ids[hit_st].insert(piece_id);
                    }

                    rayidx = pc_endidx;
                }
            //}
            projs.clear();
            samples.clear();
        }

        int num_bad_outlines = 0, num_outlines = 0;
        std::vector<st_outlines_entry> ret;
        for (auto& [street, st_piece_ids] : street_piece_ids)
        {
            assert(!st_piece_ids.empty());

            auto& entry = ret.emplace_back();
            entry.street = street;

            for (auto st_pid : st_piece_ids)
            {
                auto* const init_piece = &pieces[st_pid];
                if (init_piece->joined) {
                    continue;
                }

                // Join pieces by ID to build outline
                direction outline_start_dir, outline_end_dir;
                auto& outline = entry.outlines.emplace_back();
                {
                    // go to the start of the chain
                    auto* cur_piece = init_piece;
                    while (true)
                    {
                        int prev_segidx = cur_piece->seg->prev_seg_gidx;
                        auto* prev_seg = prev_segidx != -1 ? &m_footpath_segs[prev_segidx] : nullptr;

                        if (cur_piece->start.id == -1 || !prev_seg || fseg_piece_ids[prev_seg].empty()) {
                            break;
                        }
                        int prev_piece_id = fseg_piece_ids[prev_seg].back();
                        if (!st_piece_ids.contains(prev_piece_id)) {
                            break;
                        }
                        auto* prev_piece = &pieces[prev_piece_id];
                        if (prev_piece->end.id != cur_piece->start.id) {
                            break;
                        }
                        cur_piece = prev_piece;
                    }

                    outline.start_proj = cur_piece->start_proj;
                    outline_start_dir = path_seg_pt_rel_dir(street, 
                        cur_piece->start_proj.seg_pidx, cur_piece->start.vert);
                    outline.dir = outline_start_dir;

                    // extend chain forwards to init piece
                    while (cur_piece != init_piece)
                    {
                        cur_piece->joined = true;
                        outline.nodes.push_back(cur_piece->start);

                        auto* next_seg = &m_footpath_segs[cur_piece->seg->next_seg_gidx];
                        cur_piece = &pieces[fseg_piece_ids[next_seg].front()];

                        // every middle piece should be a complete segment
                        assert(cur_piece == init_piece ||
                            (fseg_piece_ids[cur_piece->seg].size() == 1 &&
                                cur_piece->start.id != -1 && cur_piece->end.id != -1));
                    }

                    // extend chain forwards
                    while (true)
                    {
                        cur_piece->joined = true;
                        outline.nodes.push_back(cur_piece->start);

                        int next_segidx = cur_piece->seg->next_seg_gidx;
                        auto* next_seg = next_segidx != -1 ? &m_footpath_segs[next_segidx] : nullptr;

                        if (cur_piece->end.id == -1 || !next_seg || fseg_piece_ids[next_seg].empty()) {
                            break;
                        }
                        int next_piece_id = fseg_piece_ids[next_seg].front();
                        if (!st_piece_ids.contains(next_piece_id)) {
                            break;
                        }
                        auto* next_piece = &pieces[next_piece_id];
                        if (next_piece->start.id != cur_piece->end.id) {
                            break;
                        }
                        cur_piece = next_piece;
                    }

                    outline.nodes.push_back(cur_piece->end);
                    assert(outline.nodes.size() >= 2);

                    outline.end_proj = cur_piece->end_proj;
                    outline_end_dir = path_seg_pt_rel_dir(street, 
                        cur_piece->end_proj.seg_pidx, cur_piece->end.vert);

                    num_outlines++;
                }

                assert(outline_start_dir != DIR_UNDEF && outline_end_dir != DIR_UNDEF);

                // this means that the footpath crossed the street, 
                // which should be impossible for real streets/sidewalks
                if (outline_start_dir != outline_end_dir) {
                    entry.outlines.pop_back();
                    num_bad_outlines++;
                    continue;
                }

                // ensure start_proj is before end_proj
                auto& sp = outline.start_proj, &ep = outline.end_proj;
                if (sp.seg_pidx > ep.seg_pidx ||
                    (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param))
                {
                    std::reverse(outline.nodes.begin(), outline.nodes.end());
                    std::swap(sp, ep);
                }
            }

            // if all outlines were bad, remove (very unlikely)
            if (entry.outlines.empty()) {
                ret.pop_back();
            }
        }
    
        if (num_bad_outlines > 0) {
            logWARNING("% bad street outlines: %d/%d\n", num_bad_outlines, num_outlines);
        }
        return ret;
    }

    struct ray_hit
    {
        const tree_path_seg* fseg;
        glm::dvec2 point;

        static constexpr ray_hit no_hit() {
            return { .fseg = nullptr, .point = glm::dvec2(0.0) };
        }
    };

    ray_hit get_ft_hit(const ray2d& ray, const tree_path_seg* src_tseg, double eps)
    {
        struct qdata
        {
            double sqdist;
            glm::dvec2 point;
        };
        
        auto dist_range = param_range{ 0.0, 20.0 };

        qdata hit_segdata{};
        tree_path_seg* hit_seg = nullptr;
        
        bool hit = m_seg_tree.query_nearest(ray, dist_range,
            [&](tree_path_seg* cand_tseg, qdata& out_qdata) -> bool
            {
                if (cand_tseg->path == src_tseg->path) {
                    return false;
                }

                // check if footpath and street are somewhat parallel
                if (cand_tseg->way->type == WAY_TYPE_FOOTWAY) {
                    double angle_bw = min_angle_bw_unitdirs(src_tseg->udir, cand_tseg->udir);
                    if (std::abs(glm::degrees(angle_bw)) > 45.0) {
                        return false;
                    }
                }

                seg_inter_result res;
                segment cand_seg = { cand_tseg->start.vert, cand_tseg->end.vert };
                segment ray_seg = { ray.at_param(dist_range.min()), ray.at_param(dist_range.max()) };

                if (seg_intersect(cand_seg, ray_seg, res, eps)) {
                    out_qdata.sqdist = vec_sqlength(res.point - ray.origin);
                    out_qdata.point = res.point;
                    return true;
                }
                return false;
            },
            hit_seg, hit_segdata);

        if (hit && hit_seg->way->type == WAY_TYPE_FOOTWAY) {
            return {
                .fseg = hit_seg,
                .point = hit_segdata.point
            };
        }

        return ray_hit::no_hit();
    };

    std::vector<st_outlines_entry> get_all_st_ft_outlines_from_streets(double eps)
    {
        static constexpr double RAYCAST_INTERVAL = 1; // meters

        struct outline_piece
        {
            std::vector<outline_node> nodes;
            outline_proj::data start_proj, end_proj;
            direction dir;
            int sseg_gidx; // street segment global index
        };

        std::vector<std::pair<double, glm::dvec2>> ray_origins;
        std::vector<ray_hit> ray_hits;

        std::vector<st_outlines_entry> ret;
        for (auto& sseg : m_street_segs)
        // Map from street path to all outline pieces for that street
        types::unord_flat_map<const way_net::path*, std::vector<outline_piece>> street_pieces_map;

        for (size_t sseg_gidx = 0; sseg_gidx < m_street_segs.size(); ++sseg_gidx)
        {
            auto& sseg = m_street_segs[sseg_gidx];
            
            glm::dvec2 sseg_vec = sseg.end.vert - sseg.start.vert;
            glm::dvec2 sseg_perp_dir = glm::normalize(vec_perp(sseg_vec));

            int num_rays = std::max(1, int(std::ceil(sseg.length / RAYCAST_INTERVAL - eps))) + 1;
            for (int i = 0; i < num_rays; ++i) 
            {
                double t = double(i) / (num_rays - 1);
                ray_origins.push_back({ t, sseg.start.vert + t * sseg_vec });
            }

            std::array<std::pair<direction, glm::dvec2>, 2> ray_dirs = {{
                { DIR_LEFT, sseg_perp_dir },
                { DIR_RIGHT, -sseg_perp_dir }
            }};

            for (const auto& dir : ray_dirs)
            {
                if (sseg.way->id == 634716097) {
                    logMESSAGE("debug");
                }


                for (int i = 0; i < num_rays; ++i) {
                    ray2d ray{ .origin = ray_origins[i].second, .dir = dir.second };
                    ray_hits.push_back(get_ft_hit(ray, &sseg, eps));
                }

                if (sseg.way->id == 634716097) {
                    logMESSAGE("debug");
                }


                int rayidx = 0;
                while (rayidx < num_rays)
                {
                    // get largest outlines
                    int pc_startidx = rayidx;
                    while (pc_startidx < num_rays && !ray_hits[pc_startidx].fseg) {
                        pc_startidx++;
                    }
                    int pc_endidx = pc_startidx + 1;
                    while (pc_endidx < num_rays && ray_hits[pc_endidx].fseg &&
                        ray_hits[pc_endidx].fseg->path == ray_hits[pc_startidx].fseg->path) {
                        pc_endidx++;
                    }

                    if (pc_startidx < num_rays && pc_endidx - pc_startidx > 1)
                    {
                        /*outline_piece piece;
                        piece.seg = &fseg;

                        if (pc_startidx == 0) {
                            piece.start = outline_node::osm(fseg.start);
                        }
                        else {
                            piece.start.id = -1;
                            piece.start.vert = samples[pc_startidx];
                        }
                        if (pc_endidx == num_rays) {
                            piece.end = outline_node::osm(fseg.end);
                        }
                        else {
                            piece.end.id = -1;
                            piece.end.vert = samples[pc_endidx - 1];
                        }

                        piece.start_proj = ray_hits[pc_startidx].proj;
                        piece.end_proj = ray_hits[pc_endidx - 1].proj;

                        int piece_id = int(pieces.size());
                        auto* hit_st = ray_hits[pc_startidx].street;

                        pieces.push_back(piece);
                        fseg_piece_ids[&fseg].push_back(piece_id);
                        street_piece_ids[hit_st].insert(piece_id);*/

                        st_outlines_entry* entry;
                        if (!ret.empty() && ret.back().street == sseg.path) {
                            entry = &ret.back();
                        } else {
                            entry = &ret.emplace_back();
                            entry->street = sseg.path;
                        }

                        st_outline outline;
                        outline.dir = dir.first;
                        outline.start_proj = {
                        outline_piece piece;
                        piece.dir = dir.first;
                        piece.sseg_gidx = int(sseg_gidx);
                        piece.start_proj = {
                            .pt_param = ray_origins[pc_startidx].first,
                            .seg_pidx = sseg.pathidx
                        };
                        outline.end_proj = {
                        piece.end_proj = {
                            .pt_param = ray_origins[pc_endidx - 1].first,
                            .seg_pidx = sseg.pathidx
                        };
                        for (int ri = pc_startidx; ri < pc_endidx; ++ri) {
                            outline.nodes.push_back({
                            piece.nodes.push_back({
                                .id = -1,
                                .vert = ray_hits[ri].point
                            });
                        }

                        // ensure start_proj is before end_proj
                        auto& sp = outline.start_proj, & ep = outline.end_proj;
                        auto& sp = piece.start_proj, & ep = piece.end_proj;
                        if (sp.seg_pidx > ep.seg_pidx ||
                            (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param))
                        {
                            std::reverse(outline.nodes.begin(), outline.nodes.end());
                            std::reverse(piece.nodes.begin(), piece.nodes.end());
                            std::swap(sp, ep);
                        }

                        entry->outlines.push_back(std::move(outline));
                        street_pieces_map[sseg.path].push_back(std::move(piece));
                    }

                    rayidx = pc_endidx;
                }

                ray_hits.clear();
            }

            ray_origins.clear();
            ray_hits.clear();
        }

        // Now join adjacent pieces for each street
        std::vector<st_outlines_entry> ret;
        for (auto& [street, pieces] : street_pieces_map)
        {
            if (pieces.empty()) continue;

            auto& entry = ret.emplace_back();
            entry.street = street;

            // Sort pieces by direction, then segment index, then start parameter
            std::sort(pieces.begin(), pieces.end(), 
                [](const outline_piece& lhs, const outline_piece& rhs)
            {
                if (lhs.dir != rhs.dir) return lhs.dir < rhs.dir;
                if (lhs.start_proj.seg_pidx != rhs.start_proj.seg_pidx) {
                    return lhs.start_proj.seg_pidx < rhs.start_proj.seg_pidx;
                }
                return lhs.start_proj.pt_param < rhs.start_proj.pt_param;
            });

            // Join adjacent pieces with same direction
            std::vector<bool> piece_used(pieces.size(), false);
            for (size_t i = 0; i < pieces.size(); ++i)
            {
                if (piece_used[i]) continue;

                st_outline outline;
                outline.dir = pieces[i].dir;
                outline.start_proj = pieces[i].start_proj;
                outline.end_proj = pieces[i].end_proj;
                outline.nodes = pieces[i].nodes;
                piece_used[i] = true;

                // Try to extend forward by joining adjacent pieces
                bool extended = true;
                while (extended)
                {
                    extended = false;
                    for (size_t j = i + 1; j < pieces.size(); ++j)
                    {
                        if (piece_used[j] || pieces[j].dir != outline.dir) continue;

                        // Check if pieces are adjacent
                        bool adjacent = false;
                        
                        // Same segment - check if they're consecutive in parameter
                        if (pieces[j].start_proj.seg_pidx == outline.end_proj.seg_pidx &&
                            std::abs(pieces[j].start_proj.pt_param - outline.end_proj.pt_param) < 0.01) 
                        {
                            adjacent = true;
                        }
                        // Adjacent segments - check if end is at segment boundary
                        else if (pieces[j].sseg_gidx == m_street_segs[pieces[i].sseg_gidx].next_seg_gidx &&
                                 pieces[j].start_proj.seg_pidx == outline.end_proj.seg_pidx + 1 &&
                                 outline.end_proj.pt_param > 0.99 && 
                                 pieces[j].start_proj.pt_param < 0.01) 
                        {
                            adjacent = true;
                        }

                        if (adjacent)
                        {
                            // Join the pieces
                            outline.nodes.insert(outline.nodes.end(), 
                                pieces[j].nodes.begin(), pieces[j].nodes.end());
                            outline.end_proj = pieces[j].end_proj;
                            piece_used[j] = true;
                            extended = true;
                            break;
                        }
                    }
                }

                if (outline.nodes.size() >= 2) {
                    entry.outlines.push_back(std::move(outline));
                }
            }

            if (entry.outlines.empty()) {
                ret.pop_back();
            }
        }

        return ret;
    }

    // All outlines must belong to the same street and fall on the same side (dir).
    std::vector<outline_node> fill_st_outline_holes(const way_net::path* street, 
        std::vector<const st_outline*>& outlines, direction outlines_dir, double eps)
    {
        assert(outlines_dir == DIR_LEFT || outlines_dir == DIR_RIGHT);

        std::sort(outlines.begin(), outlines.end(), [](auto* lhs, auto* rhs)
        {
            auto& lp = lhs->start_proj, &rp = rhs->start_proj;
            if (lp.seg_pidx == rp.seg_pidx) {
                return lp.pt_param < rp.pt_param;
            } else {
                return lp.seg_pidx < rp.seg_pidx;
            }
        });

        std::vector<outline_node> ret;
        auto add_genpoint = [&](glm::dvec2 vert) {
            ret.push_back({ .id = -1, .vert = vert });
        };

        // repeated usage of path_seg and path_seg_width should be optimized out
        auto stseg_point = [&](int segidx, double param) {
            segment seg = path_seg(street, segidx);
            return seg_at_param(seg, param);
        };

        auto stseg_normal = [&](int segidx) {
            segment seg = path_seg(street, segidx);
            double width = path_seg_width(street, segidx);
            return seg_normal(seg, outlines_dir, width / 2);
        };

        const st_outline* outline; int outline_idx = 0;
        int hole_start_segidx = 0; double hole_start_param = 0.0;
        do 
        {
            int hole_end_segidx; double hole_end_param;

            if (outline_idx == outlines.size()) {
                outline = nullptr;
                hole_end_segidx = int(street->nodes.size() - 2);
                hole_end_param = 1.0;
            } else {
                outline = outlines[outline_idx];
                hole_end_segidx = outline->start_proj.seg_pidx;
                hole_end_param = outline->start_proj.pt_param;
            }

            //assert(hole_start_segidx <= hole_end_segidx);
            if (hole_start_segidx > hole_end_segidx) {
                return {};
            }

            // generate points between hole start point and hole end point
            if (hole_start_segidx < hole_end_segidx)
            {
                glm::dvec2 hole_start = stseg_point(hole_start_segidx, hole_start_param) + stseg_normal(hole_start_segidx);
                glm::dvec2 hole_end = stseg_point(hole_end_segidx, hole_end_param) + stseg_normal(hole_end_segidx);

                add_genpoint(hole_start);
                for (int segidx = hole_start_segidx; segidx < hole_end_segidx; ++segidx)
                {
                    segment cur_seg = path_seg(street, segidx);
                    segment next_seg = path_seg(street, segidx + 1);
                    double width_diff = path_seg_width(street, segidx) - path_seg_width(street, segidx + 1);

                    if (min_angle_bw_segs(cur_seg, next_seg) > glm::radians(5.0) || std::abs(width_diff) > eps) {
                        add_genpoint(cur_seg.second + stseg_normal(segidx));
                        add_genpoint(cur_seg.second + stseg_normal(segidx + 1));
                    }
                }
                add_genpoint(hole_end);
            }
            else if (hole_start_segidx == hole_end_segidx)
            {
                //assert(hole_start_param <= hole_end_param);
                if (hole_start_param > hole_end_param) {
                    return {};
                }

                if (hole_start_param < hole_end_param)
                {
                    glm::dvec2 norm = stseg_normal(hole_start_segidx);
                    add_genpoint(stseg_point(hole_start_segidx, hole_start_param) + norm);
                    add_genpoint(stseg_point(hole_start_segidx, hole_end_param) + norm);
                }
            }

            if (outline) {
                for (const auto& node : outline->nodes) {
                    ret.push_back(node);
                }
                hole_start_segidx = outline->end_proj.seg_pidx;
                hole_start_param = outline->end_proj.pt_param;
                outline_idx++;
            }
        } 
        while (outline);

        return ret;
    }

private:
    std::vector<tree_path_seg> m_footpath_segs;
    std::vector<tree_path_seg> m_street_segs;
    aabb_tree2d<tree_path_seg*> m_seg_tree;
    const aabb_tree2d<mesh_builder::building*>& m_bldg_tree;
};


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

static bool gen_outline_drawdata(draw_datad& dd, const std::vector<street_outline_builder::outline_node>& outline)
{
    std::vector<glm::dvec2> outline_verts;

    for (size_t i = 0; i < outline.size(); ++i) {
        outline_verts.push_back(outline[i].vert);
    }

    orient_t outline_orient = polygon_orient(outline_verts);
    if (outline_orient == ORIENT_COLL) {
        logMESSAGE("Skipping outline since it has 0 area");
        //assert(false); // todo: figure out why this is failing
        return false;
    }

    auto tri_indices = polygon_triangulate(outline_verts, outline_orient);
    if (tri_indices.empty()) {
        logMESSAGE("Skipping outline since it has no triangles");
        return false;
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
    uint32_t vert_startidx = uint32_t(dd.num_verts());
    for (const auto& point : outline_verts) {
        dd.add_vertex(point.x, point.y, 0.0);
    }
    for (size_t i = 0; i < tri_indices.size(); i += 3) {
        dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
    }
    //}
    return true;
}

bool mesh_builder::gen_street_drawdata(std::vector<draw_datad>& drawdata, const aabb_tree2d<building*>* bldg_tree_ptr)
{
    auto tbegin = clk::now();

    auto tbegin_create = clk::now();
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
    auto tend_create = clk::now();

    logMESSAGE("Street network creation took: %s", time_str(tend_create - tbegin_create).c_str());
    
    auto tbegin_paths = clk::now();
    auto all_paths = get_all_paths_bw_intersections(network);
    auto tend_paths = clk::now();

    logMESSAGE("Street path extraction took: %s", time_str(tend_paths - tbegin_paths).c_str());

    constexpr double eps = 1e-9;
    street_outline_builder st_outline_builder(all_paths, *bldg_tree_ptr);
    auto st_outlines_map = st_outline_builder.get_outlines(eps);
    

    draw_datad footpath_dd{ 
        .name = "footpaths",
        .color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f)
    };
    draw_datad street_dd{ 
        .name = "streets",
        .color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f)
    };
    //draw_datad debug_dd{
    //    .name = "debug",
    //    .color = glm::vec4(1.0f, 0.f, 0.f, 1.f)
    //};

    for (const auto& footpath : all_paths.footpaths) {
        gen_path_drawdata(footpath_dd, footpath, eps);
    }

    tim::high_resolution_clock::duration ttotal_outlines(0);

    for (const auto& street : all_paths.streets) 
    {
        auto outline_map_itr = st_outlines_map.find(&street);

        if (outline_map_itr == st_outlines_map.end()) {
            gen_path_drawdata(street_dd, street, eps);
        } else {
            auto tbegin = clk::now();
            gen_outline_drawdata(street_dd, outline_map_itr->second);
            auto tend = clk::now();
            ttotal_outlines += (tend - tbegin);
        }
    }

    logMESSAGE("Street outline triangulation time: %s", time_str(ttotal_outlines).c_str());

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Street gen took: %s", time_str(tend - tbegin).c_str());

    return true;
}
