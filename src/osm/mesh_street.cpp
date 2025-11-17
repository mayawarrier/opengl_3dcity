#include <algorithm>
#include <span>
#include <array>
#include <unordered_map>
#include <iterator>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/unordered/unordered_flat_map.hpp>
#include <boost/container/flat_set.hpp>

#include "containers/way_network.hpp"
#include "containers/aabb_tree.hpp"
#include "geom.hpp"
#include "mesh.hpp"


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

// Strategy:
// Snap the street segments to the footpaths, if they are within a certain distance and angular tolerance
// Consider left and right outlines of the street separately.
// If footpath is not present, just draw the outline normally, by using estimated width from OSM tags.
// Smoothly interpolate between street segments with different widths.

//todo: I can start cleaning up now and reimplementing parts of the code
// some footpaths are inside area-mapped highways, so I need to check if the footpath is inside an area and remove it, just generate the area
// some footpaths are at different heights than the street! These should be ignored or drawn appropriately
// some footpaths obscure other footpaths, so I need to check if the footpath is obscured by a building or another footpath

class st_outline_builder
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

    st_outline_builder(const way_net_paths& all_paths, 
        const aabb_tree2d<mesh_builder::building*>& bldg_tree, 
        std::function<void(const char*, clk::duration)> step_done_func) :
        m_all_paths(all_paths), 
        m_bldg_tree(bldg_tree), 
        m_step_done(std::move(step_done_func))
    {
        auto tbegin_prep = clk::now();

        m_footpath_segs = get_all_path_segments(all_paths.footpaths);
        m_street_segs = get_all_path_segments(all_paths.streets);

        size_t tobj_idx = 0;
        const size_t nsegs = m_footpath_segs.size() + m_street_segs.size();
        buffer<tree_path_seg*> tree_objects(nsegs, buffer_overwrite);

        for (auto& seg : m_footpath_segs) {
            tree_objects.ptr[tobj_idx++] = &seg;
        }
        for (auto& seg : m_street_segs) {
            tree_objects.ptr[tobj_idx++] = &seg;
        }
        m_seg_tree = aabb_tree2d<tree_path_seg*>::create_unsafe(tree_objects.span());

        auto tend_prep = clk::now();     
        m_step_done("Outline extraction prep", tend_prep - tbegin_prep);
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
        struct pdata {
            double pt_param;
            int seg_pidx;
        } data;

        static constexpr outline_proj no_proj() {
            return {
                .street = nullptr,
                .data = { .pt_param = -1.0, .seg_pidx = -1 }
            };
        }
    };

    outline_proj get_street_proj(const glm::dvec2& sample, const tree_path_seg* src_tseg, double eps)
    {
        struct tree_qdata
        {
            double sqdist;
            double pt_param;
        };

        auto dist_range = param_range{ 0.0, 20.0 };

        tree_qdata hit_segdata{};
        tree_path_seg* hit_seg = nullptr;

        bool hit = m_seg_tree.query_nearest(sample, dist_range, 
            [&](tree_path_seg* cand_tseg, tree_qdata& out_qdata) -> bool
            {
                if (cand_tseg->path == src_tseg->path) {
                    return false;
                }

                // check if footpath and street are somewhat parallel
                if (cand_tseg->way->type == WAY_TYPE_STREET) {
                    double angle_bw = min_angle_bw_unitdirs(src_tseg->udir, cand_tseg->udir);
                    if (std::abs(glm::degrees(angle_bw)) > 45.0) {
                        return false;
                    }
                }

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
            }, 
            hit_seg, hit_segdata);

        if (hit && hit_seg->way->type == WAY_TYPE_STREET) {
            return {
                .street = hit_seg->path,
                .data = {
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
        outline_proj::pdata start_proj, end_proj;
        direction dir; // wrt to street
    };

    struct st_outlines_entry
    {
        const way_net::path* street;
        types::small_vector<st_outline, 4> outlines;
    };

    // Get street outlines from nearby footpaths
    std::vector<st_outlines_entry> get_all_st_ft_outlines(double eps)
    {
        static constexpr double SAMPLE_INTERVAL = 1; // meters

        struct outline_piece
        {
            outline_node start, end;
            outline_proj::pdata start_proj, end_proj;
            const tree_path_seg* seg;
            bool joined = false;
        };

        std::vector<outline_piece> pieces;
        types::unord_flat_map<const tree_path_seg*, types::small_vector<int, 2>> fseg_piece_ids;
        types::unord_flat_map<const way_net::path*, types::small_flat_set<int, 2>> street_piece_ids;

        struct sample_data
        {
            glm::dvec2 point;
            outline_proj proj;
        };
        std::vector<sample_data> samples;

        for (auto& fseg : m_footpath_segs)
        {
            glm::dvec2 fseg_vec = fseg.end.vert - fseg.start.vert;
            glm::dvec2 fseg_perp_dir = glm::normalize(vec_perp(fseg_vec));

            int num_samples = std::max(1, int(std::ceil(fseg.length / SAMPLE_INTERVAL - eps))) + 1;
            for (int i = 0; i < num_samples; ++i) {
                samples.push_back({ .point = fseg.start.vert + (double(i) / (num_samples - 1)) * fseg_vec });
            }

            //for (direction dir : { DIR_LEFT, DIR_RIGHT })
            //{
                for (int i = 0; i < num_samples; ++i) {
                    samples[i].proj = get_street_proj(samples[i].point, &fseg, eps);
                }

                int rayidx = 0;
                while (rayidx < num_samples)
                {
                    // get largest piece that hits the same street
                    int pc_startidx = rayidx;
                    while (pc_startidx < num_samples && !samples[pc_startidx].proj.street) {
                        pc_startidx++;
                    }
                    int pc_endidx = pc_startidx + 1;
                    while (pc_endidx < num_samples && samples[pc_endidx].proj.street &&
                        samples[pc_endidx].proj.street == samples[pc_startidx].proj.street) {
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
                            piece.start.vert = samples[pc_startidx].point;
                        }
                        if (pc_endidx == num_samples) {
                            piece.end = outline_node::osm(fseg.end);
                        } else {
                            piece.end.id = -1;
                            piece.end.vert = samples[pc_endidx - 1].point;
                        }

                        piece.start_proj = samples[pc_startidx].proj.data;
                        piece.end_proj = samples[pc_endidx - 1].proj.data;

                        int piece_id = int(pieces.size());
                        auto* hit_st = samples[pc_startidx].proj.street;

                        pieces.push_back(piece);
                        fseg_piece_ids[&fseg].push_back(piece_id);
                        street_piece_ids[hit_st].insert(piece_id);
                    }

                    rayidx = pc_endidx;
                }
            //}

            // does not free capacity
            samples.clear();
        }

        using st_pieces_itr = decltype(street_piece_ids)::iterator;

        // Join pieces by ID to build outline.
        auto join_outline_from_pieces = [&](const st_pieces_itr& st_itr, 
            outline_piece* seed_piece, st_outline& outline, direction& out_start_dir, direction& out_end_dir)
        {
            auto& street = st_itr->first;
            auto& st_piece_ids = st_itr->second;

            const outline_piece* start_piece, *end_piece;

            // go to the start of the chain
            auto* cur_piece = seed_piece;
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
            start_piece = cur_piece;

            // extend chain forwards to seed piece
            while (cur_piece != seed_piece)
            {
                cur_piece->joined = true;
                outline.nodes.push_back(cur_piece->start);

                auto* next_seg = &m_footpath_segs[cur_piece->seg->next_seg_gidx];
                cur_piece = &pieces[fseg_piece_ids[next_seg].front()];

                // every middle piece should be a complete segment
                assert(cur_piece == seed_piece ||
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
            end_piece = cur_piece;

            outline.nodes.push_back(cur_piece->end);
            assert(outline.nodes.size() >= 2);

            auto pt_sseg_rel_dir = [&](const glm::dvec2& pt, int segpidx) -> direction
            {
                auto seg = path_seg(street, segpidx);
                auto pto = orient(seg.first, seg.second, pt);
                switch (pto)
                {
                case ORIENT_CCW:  return DIR_LEFT;
                case ORIENT_COLL: return DIR_UNDEF;
                case ORIENT_CW:   return DIR_RIGHT;
                default:
                    assert(false);
                    return DIR_UNDEF;
                }
            };

            outline.start_proj = start_piece->start_proj;
            outline.end_proj = end_piece->end_proj;
            out_start_dir = pt_sseg_rel_dir(start_piece->start.vert, start_piece->start_proj.seg_pidx);
            out_end_dir = pt_sseg_rel_dir(end_piece->end.vert, end_piece->end_proj.seg_pidx);

            outline.dir = out_start_dir;
        };

        int num_bad_outlines = 0, num_outlines = 0;
        
        std::vector<st_outlines_entry> ret;
        for (auto stpcs_itr = street_piece_ids.begin(); stpcs_itr != street_piece_ids.end(); ++stpcs_itr)
        {
            assert(!stpcs_itr->second.empty());

            auto& entry = ret.emplace_back();
            entry.street = stpcs_itr->first;

            for (auto st_pid : stpcs_itr->second)
            {
                auto* const init_piece = &pieces[st_pid];
                if (init_piece->joined) {
                    continue;
                }

                direction start_dir, end_dir;
                st_outline& outline = entry.outlines.emplace_back();
                join_outline_from_pieces(stpcs_itr, init_piece, outline, start_dir, end_dir);
                num_outlines++;

                // this means that the footpath crossed the street,
                // which should be impossible for real streets/sidewalks
                if (start_dir == DIR_UNDEF || end_dir == DIR_UNDEF || start_dir != end_dir) {
                    entry.outlines.pop_back();
                    num_bad_outlines++;
                    continue;
                }

                // align outline with street direction
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
            logWARNING("Bad outlines: %d/%d", num_bad_outlines, num_outlines);
        }
        return ret;
    }

    // All outlines must belong to the same street and fall on the same side (dir).
    std::vector<outline_node> fill_st_outline_holes_for_dir(const way_net::path* street, 
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
        auto add_genpoint = [&](const glm::dvec2& vert) {
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

        int outline_idx = 0;
        const st_outline* outline = nullptr;
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

    street_outlines_t fill_all_st_outline_holes(const std::vector<st_outlines_entry>& st_entries, double eps)
    {
        street_outlines_t ret;
        for (auto& st_entry : st_entries)
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
                default: assert(false); break;
                }
            }

            auto lt_outline = fill_st_outline_holes_for_dir(street, left_outlines, DIR_LEFT, eps);
            auto rt_outline = fill_st_outline_holes_for_dir(street, right_outlines, DIR_RIGHT, eps);

            if (lt_outline.empty() || rt_outline.empty()) {
                continue;
            }

            glm::dvec4 dists = {
                vec_sqlength(lt_outline.back().vert - rt_outline.front().vert),
                vec_sqlength(lt_outline.back().vert - rt_outline.back().vert),
                vec_sqlength(lt_outline.front().vert - rt_outline.front().vert),
                vec_sqlength(lt_outline.front().vert - rt_outline.back().vert),
            };

            int min_distidx = vec_argmin(dists);
            bool reverse_lt = get_bit(min_distidx, 1); // nasty!
            bool reverse_rt = get_bit(min_distidx, 0);

            if (reverse_lt) { std::reverse(lt_outline.begin(), lt_outline.end()); }
            if (reverse_rt) { std::reverse(rt_outline.begin(), rt_outline.end()); }

            lt_outline.insert(lt_outline.end(), rt_outline.begin(), rt_outline.end());

            ret[street] = std::move(lt_outline);
        }

        return ret;
    }

public:
    street_outlines_t build_outlines(double eps)
    {
        auto tbegin_outlines = clk::now();
        auto entries = get_all_st_ft_outlines(eps);
        auto tend_outlines = clk::now();

        m_step_done("Outline extraction", tend_outlines - tbegin_outlines);

        auto tbegin_holefill = clk::now();
        auto ret = fill_all_st_outline_holes(entries, eps);
        auto tend_holefill = clk::now();

        m_step_done("Outline hole filling", tend_holefill - tbegin_holefill);

        return ret;
    }

    static constexpr int num_steps() {
        return 3;
    }

private:
    const way_net_paths& m_all_paths;
    std::vector<tree_path_seg> m_footpath_segs;
    std::vector<tree_path_seg> m_street_segs;
    aabb_tree2d<tree_path_seg*> m_seg_tree;
    const aabb_tree2d<mesh_builder::building*>& m_bldg_tree;
    std::function<void(const char*, clk::duration)> m_step_done;
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

static bool gen_outline_drawdata(draw_datad& dd, const std::vector<st_outline_builder::outline_node>& outline)
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

    uint32_t vert_startidx = uint32_t(dd.num_verts());
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

    int CUR_STEP = 1;
    constexpr int NUM_STEPS = 4 + st_outline_builder::num_steps();

    auto step_done = [&](const char* msg, clk::duration dur) {
        logMESSAGE("  [%d/%d] %s: %s", CUR_STEP, NUM_STEPS, msg, time_str(dur).c_str());
        CUR_STEP++;
    };
    
    logMESSAGE("-----------------------------------------------");
    logMESSAGE("Generating streets...");
    logMESSAGE("%zu ways, %zu nodes", m_highways.size(), m_num_highway_nodes);
    
    auto tbegin = clk::now();

    auto tbegin_net = clk::now();
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
    auto tend_net = clk::now();

    step_done("Network creation", tend_net - tbegin_net);
    
    auto tbegin_paths_ex = clk::now();
    auto all_paths = get_all_paths_bw_intersections(network);
    auto tend_paths_ex = clk::now();

    step_done("Path extraction", tend_paths_ex - tbegin_paths_ex);

    st_outline_builder st_builder(all_paths, *bldg_tree_ptr, step_done);
    auto st_outline_map = st_builder.build_outlines(eps);

    draw_datad footpath_dd { .name = "footpaths", .color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f) };
    draw_datad street_dd { .name = "streets", .color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f) };
    //draw_datad debug_dd { .name = "debug", .color = glm::vec4(1.0f, 0.f, 0.f, 1.f) };

    auto tbegin_outlines = clk::now();
    for (auto& [street, outline] : st_outline_map) {
        if (!gen_outline_drawdata(street_dd, outline)) {
            gen_path_drawdata(street_dd, *street, eps);
        }
    }
    auto tend_outlines = clk::now();

    step_done("Outline triangulation", tend_outlines - tbegin_outlines);

    auto tbegin_paths = clk::now();
    for (const auto& footpath : all_paths.footpaths) {
        gen_path_drawdata(footpath_dd, footpath, eps);
    } 
    for (const auto& street : all_paths.streets) {
        if (!st_outline_map.contains(&street)) {
            gen_path_drawdata(street_dd, street, eps);
        }
    }
    auto tend_paths = clk::now();

    step_done("Path triangulation", tend_paths - tbegin_paths);

    uint32_t num_tris = street_dd.num_tris() + footpath_dd.num_tris();
    uint32_t num_verts = street_dd.num_verts() + footpath_dd.num_verts();

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Generated %u tris and %u vertices in %s", num_tris, num_verts, time_str(tend - tbegin).c_str());
    logMESSAGE("-----------------------------------------------\n");

    return true;
}
