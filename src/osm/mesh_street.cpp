#include <algorithm>
#include <utility>
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
        .layer = info.layer,
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
    return { path->nodes[idx].vert(), path->nodes[idx + 1].vert() };
}

static inline double path_seg_width(const way_net::path* path, int idx) {
    // +1 because in_way is null for the first node
    return path->nodes[idx + 1].in_way->width;
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
    //
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

    st_outline_builder(
        const way_net& network,
        const way_net_paths& all_paths, 
        const aabb_tree2d<mesh_builder::building*>& bldg_tree, 
        std::function<void(const char*, clk::duration)> step_done_func) :
        m_network(network),
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
        int pathidx;
        double length;
        glm::dvec2 udir; // unit direction
        bbox2d bbox;

        const mesh_builder::highway* way;
        const way_net::path* path;

        const osm_node& start() const { 
            return path->nodes[pathidx].osm_node; 
        }
        const osm_node& end() const { 
            return path->nodes[pathidx + 1].osm_node;
        }

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

                auto bbox = bbox2d::empty();
                bbox.extend(start.vert());
                bbox.extend(end.vert());

                double length = glm::length(end.vert() - start.vert());

                ret.push_back({
                    .pathidx = i,
                    .length = length,
                    .udir = (end.vert() - start.vert()) / length,
                    .bbox = bbox,
                    .way = end.in_way,
                    .path = &path,
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

    // I have an idea. First go through all the street segments. Shoot rays
// on both sides. Use that to mark the points on the footpath segments
// that definitely have a 90 projection on to a given street. Go through
// the footpaths now, gathering points that came from the same street
// and simply join them together to form the set of outlines.
// I will know which side they came from and the originating point.
// So what was the problem before? Only if I am certain should I rewrite.
// I think the issue before was I didn't want to go through the street segments twice,
// so I didn't think of this strategy?
// but I think I need to, to separate the left and right sides easily.
// Another problem before was that I couldn't create a piece that started at one
// footpath segment and ended at another, because I was using repeated street rays from
// the same street to join footpath pieces. But now I can iterate through footpaths so
// it should be a non-issue since I can easily identify which pieces belong to the same footpath
// and can therefore be joined.

    // todo: return results from both sides of the sample footpath, using only one point query!
    //outline_proj get_street_proj(const glm::dvec2& sample, const tree_path_seg* src_tseg, double eps)
    //{
    //    struct tree_qdata
    //    {
    //        double sqdist;
    //        double pt_param;
    //    };
    //    tree_qdata hit_segdata{};
    //    tree_path_seg* hit_seg = nullptr;
    //
    //    auto dist_range = param_range{ 0.0, 20.0 };
    //
    //    bool hit = m_seg_tree.query_nearest(sample, dist_range, 
    //        [&](tree_path_seg* cand_tseg, tree_qdata& out_qdata) -> bool
    //        {
    //            if (cand_tseg->path == src_tseg->path) {
    //                return false;
    //            }
    //
    //            // check if footpath and street are somewhat parallel
    //            if (cand_tseg->way->type == WAY_TYPE_STREET) {
    //                double angle_bw = min_angle_bw_unitvecs(src_tseg->udir, cand_tseg->udir);
    //                if (std::abs(glm::degrees(angle_bw)) > 45.0) {
    //                    return false;
    //                }
    //            }
    //
    //            segment seg = { cand_tseg->start.vert, cand_tseg->end.vert };
    //
    //            // project sample onto segment
    //            glm::dvec2 ap = sample - seg.first;
    //            glm::dvec2 ab = seg.second - seg.first;
    //            double t = glm::dot(ap, ab) / glm::dot(ab, ab);
    //
    //            // reject if out of path bounds
    //            int seg_pidx = cand_tseg->pathidx;
    //            if ((t < 0 && seg_pidx == 0) ||
    //                (t > 1.0 && seg_pidx == path_num_segs(cand_tseg->path) - 1)) {
    //                return false;
    //            }
    //
    //            double t_clamped = std::clamp(t, 0.0, 1.0);
    //            double sqdist = vec_sqlength(sample - (seg.first + t_clamped * ab));
    //
    //            if (sqdist < dist_range.min2() || sqdist > dist_range.max2()) {
    //                return false;
    //            }
    //
    //            out_qdata = {
    //                .sqdist = sqdist,
    //                .pt_param = t // keep original
    //            };
    //            return true;
    //        }, 
    //        hit_seg, hit_segdata);
    //
    //    if (hit && hit_seg->way->type == WAY_TYPE_STREET) {
    //        return {
    //            .street = hit_seg->path,
    //            .data = {
    //                .pt_param = hit_segdata.pt_param,
    //                .seg_pidx = hit_seg->pathidx,
    //            }
    //        };
    //    }
    //
    //    return outline_proj::no_proj();
    //};


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

    // I have an idea. First go through all the street segments. Shoot rays
    // on both sides. Use that to mark the points on the footpath segments
    // that definitely have a 90 projection on to a given street. Go through
    // the footpaths now, gathering points that came from the same street
    // and simply join them together to form the set of outlines.
    // I will know which side they came from and the originating point.
    // So what was the problem before? Only if I am certain should I rewrite.
    // I think the issue before was I didn't want to go through the street segments twice,
    // so I didn't think of this strategy?
    // but I think I need to, to separate the left and right sides easily.
    // Another problem before was that I couldn't create a piece that started at one
    // footpath segment and ended at another, because I was using repeated street rays from
    // the same street to join footpath pieces. But now I can iterate through footpaths so
    // it should be a non-issue since I can easily identify which pieces belong to the same footpath
    // and can therefore be joined.

    
    static constexpr double ST_FT_MAX_DIST = 20.0;
    static constexpr double ST_FT_MAX_ANGLE = glm::radians(45.0);
    
    static constexpr double ST_RAYCAST_STEP = 1; // meters
    static constexpr double ST_JUNC_ANGLE_STEP = ST_RAYCAST_STEP / ST_FT_MAX_DIST; // radians

    struct st2ft_hit
    {
        const way_net::path* footpath;
        double seg_param;
        int seg_pidx;
    };

    enum st2ft_type
    {
        ST2FT_SEGMENT,
        ST2FT_JUNCTION
    };

    std::optional<st2ft_hit> st2ft_ray(const tree_path_seg& sseg, const ray2d& ray, st2ft_type type, double eps)
    {
        struct tree_qdata
        {
            double sqdist;
            double pt_param;
        };
        tree_qdata hit_segdata{};
        tree_path_seg* hit_seg = nullptr;
        auto dist_range = param_range{ 0.0, ST_FT_MAX_DIST };

        bool hit = m_seg_tree.query_nearest(ray, dist_range,
            [&](tree_path_seg* cand_tseg, tree_qdata& out_qdata) -> bool
            {
                if (cand_tseg == &sseg || cand_tseg->way->layer != sseg.way->layer) {
                    return false;
                }
                // check if somewhat parallel
                if (type == ST2FT_SEGMENT && cand_tseg->way->type == WAY_TYPE_FOOTWAY) {
                    double angle_bw = min_angle_bw_unitvecs(sseg.udir, cand_tseg->udir);
                    if (angle_bw > ST_FT_MAX_ANGLE) {
                        return false;
                    }
                }

                segment ray_ptseg = { ray.at_param(dist_range.min()), ray.at_param(dist_range.max()) };
                segment cand_ptseg = { cand_tseg->start().vert, cand_tseg->end().vert};
                seg_inter_result inter_res;
                seg_intersect(ray_ptseg, cand_ptseg, inter_res, eps);

                if (inter_res.type == SEG_INTER_INSIDE_BOTH) {
                    out_qdata.sqdist = vec_sqlength(ray.origin - inter_res.point);
                    out_qdata.pt_param = inter_res.param_seg2;
                    return true;
                }
                return false;
            },
            hit_seg, hit_segdata);

        if (hit && hit_seg->way->type == WAY_TYPE_FOOTWAY)
        {
            return st2ft_hit{
                .footpath = hit_seg->path,
                .seg_param = hit_segdata.pt_param,
                .seg_pidx = hit_seg->pathidx
            };
        }
        return std::nullopt;
    }

    bool path_has_node(const way_net::path* path, osmium::object_id_type id) {
        return std::ranges::find_if(path->nodes, [&](auto& n) { return n.id() == id; }) != path->nodes.end();
    }

    // Get street outlines from nearby footpaths
    std::vector<st_outlines_entry> get_all_st_ft_outlines(double eps)
    {
        struct ft_hit_info
        {
            const way_net::path* street;
            direction st_side;
            st2ft_type type;
            double ft_seg_param;
            double st_param;
            int ft_seg_pidx;
            int st_seg_pidx;
        };

        types::unord_flat_map<const way_net::path*, std::vector<ft_hit_info>> ft_hits;

        auto do_st2ft_ray = [&](const ray2d& ray, st2ft_type type, 
            const tree_path_seg& sseg, direction st_side, double st_param)
        {
            auto hit = st2ft_ray(sseg, ray, type, eps);
            if (hit.has_value())
            {
                ft_hits[hit->footpath].push_back({
                    .street = sseg.path,
                    .st_side = st_side,
                    .type = type,
                    .ft_seg_param = hit->seg_param,
                    .st_param = st_param,
                    .ft_seg_pidx = hit->seg_pidx,
                    .st_seg_pidx = sseg.pathidx,
                });
            }
        };

        int sseg_gidx = 0;
        for (auto& street : m_all_paths.streets)
        {
            int street_num_segs = path_num_segs(&street);
            for (int sseg_idx = 0; sseg_idx < street_num_segs; ++sseg_idx, ++sseg_gidx)
            {
                auto& sseg = m_street_segs[sseg_gidx];

                glm::dvec2 sseg_vec = sseg.end().vert - sseg.start().vert;
                glm::dvec2 sseg_perp = vec_perp(sseg.udir);

                int sseg_num_rays = std::max(1, int(std::ceil(sseg.length / ST_RAYCAST_STEP))) + 1;
                for (int i = 0; i < sseg_num_rays; ++i)
                {
                    double t = double(i) / (sseg_num_rays - 1);
                    glm::dvec2 ray_origin = sseg.start().vert + t * sseg_vec;
                    for (direction st_side : { DIR_LEFT, DIR_RIGHT })
                    {
                        glm::dvec2 ray_dir = (st_side == DIR_LEFT) ? sseg_perp : -sseg_perp;
                        ray2d ray{ .origin = ray_origin, .dir = ray_dir };

                        do_st2ft_ray(ray, ST2FT_SEGMENT, sseg, st_side, t);
                    }
                }

                // fire rays at junctions to cover corners
                if (sseg_idx != street_num_segs - 1) {
                    glm::dvec2 sweep_start = sseg_perp;
                    glm::dvec2 sweep_end = vec_perp(m_street_segs[sseg_gidx + 1].udir);
                    
                    direction st_side = DIR_LEFT;
                    orient_t sweep_orient = orient(sseg.start().vert, sseg.end().vert, m_street_segs[sseg_gidx + 1].end().vert);
                    if (sweep_orient == ORIENT_CCW) {
                        sweep_start = -sweep_start;
                        sweep_end = -sweep_end;
                        st_side = DIR_RIGHT;
                    }
                    else if (sweep_orient == ORIENT_COLL) {
                        continue; // straight line
                    }

                    double sweep_angle = angle_bw_unitvecs(sweep_start, sweep_end);
                    int junc_num_rays = std::max(0, int(std::ceil(sweep_angle / ST_JUNC_ANGLE_STEP)) - 1); // in between only
                    if (junc_num_rays == 0) {
                        continue; // straight line
                    }

                    for (int i = 1; i <= junc_num_rays; ++i) 
                    {
                        double t = double(i) / (junc_num_rays + 1);
                        glm::dvec2 ray_dir = rotate_vec2(sweep_start, t * sweep_angle * int(sweep_orient));
                        ray2d ray{ .origin = sseg.end().vert, .dir = ray_dir};

                        do_st2ft_ray(ray, ST2FT_JUNCTION, sseg, st_side, t);
                    }
                }
            }
        }

        types::unord_flat_map<const way_net::path*, st_outlines_entry> st_outlines;

        auto add_st_outlines = [&](const way_net::path* footpath, direction st_side, std::vector<const ft_hit_info*>& hits)
        {
            std::sort(hits.begin(), hits.end(), [](auto* a, auto* b) 
            {
                if (a->ft_seg_pidx == b->ft_seg_pidx) {
                    return a->ft_seg_param < b->ft_seg_param;
                }
                return a->ft_seg_pidx < b->ft_seg_pidx;
            });

            struct st_and_outline 
            {
                const way_net::path* street;
                std::vector<outline_node> nodes;
                const ft_hit_info* start_hit, *end_hit;
            };

            auto get_ft_node = [&](int idx) -> outline_node {
                auto& n = footpath->nodes[idx];
                return outline_node::osm(n.osm_node);
            };

            auto get_endpoint = [&](const ft_hit_info* hitp) -> outline_node 
            {
                if (hitp->ft_seg_param < eps) {
                    return get_ft_node(hitp->ft_seg_pidx);
                } else if (hitp->ft_seg_param > 1.0 - eps) {
                    return get_ft_node(hitp->ft_seg_pidx + 1);
                } else {
                    glm::dvec2 ft_point = seg_at_param(path_seg(footpath, hitp->ft_seg_pidx), hitp->ft_seg_param);
                    return { .id = -1, .vert = ft_point };
                }
            };

            std::vector<st_and_outline> outlines;
            for (auto& hitp : hits) 
            {
                if (outlines.empty() || outlines.back().street != hitp->street) {
                    outlines.push_back({ 
                        .street = hitp->street, 
                        .start_hit = hitp,
                        .end_hit = nullptr
                    });
                }
                auto& outline = outlines.back();
                if (!outline.end_hit) { // first node
                    outline.nodes.push_back(get_endpoint(hitp));
                }
                // this adds intermediate nodes only, last node added later
                else for (int nidx = outline.end_hit->ft_seg_pidx + 1; nidx <= hitp->ft_seg_pidx; ++nidx) {
                    outline.nodes.push_back(get_ft_node(nidx));
                }

                outline.end_hit = hitp;
            }

            for (auto& outline : outlines) 
            {
                // last node
                auto last_point = get_endpoint(outline.end_hit);
                if (last_point.id == -1 || last_point.id != outline.nodes.back().id) {
                    outline.nodes.push_back(last_point);
                }

                st_outline ret_outline = {
                    .nodes = std::move(outline.nodes),
                    .start_proj = {
                        .pt_param = outline.start_hit->st_param,
                        .seg_pidx = outline.start_hit->st_seg_pidx
                    },
                    .end_proj = {
                        .pt_param = outline.end_hit->st_param,
                        .seg_pidx = outline.end_hit->st_seg_pidx
                    },
                    .dir = st_side
                };

                // align outline with street direction
                auto& sp = ret_outline.start_proj, &ep = ret_outline.end_proj;
                if (sp.seg_pidx > ep.seg_pidx ||
                    (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param))
                {
                    std::reverse(ret_outline.nodes.begin(), ret_outline.nodes.end());
                    std::swap(sp, ep);
                }

                st_outlines[outline.street].street = outline.street;
                st_outlines[outline.street].outlines.push_back(std::move(ret_outline));                
            }
        };

        for (const auto& [ft_path, hits] : ft_hits)
        {
            std::vector<const ft_hit_info*> left_hits, right_hits;
            for (auto& hit : hits) {
                switch (hit.st_side)
                {
                case DIR_LEFT:  left_hits.push_back(&hit); break;
                case DIR_RIGHT: right_hits.push_back(&hit); break;
                default: assert(false); break;
                }
            }

            add_st_outlines(ft_path, DIR_LEFT, left_hits);
            add_st_outlines(ft_path, DIR_RIGHT, right_hits);
        }

        //for (auto& [ft_path, hits] : ft_hits)
        //{
        //    // group hits by street and side
        //    boost::unordered::unordered_flat_map<std::pair<const way_net::path*, direction>, std::vector<ft_hit_info>> hits_by_street;
        //    for (auto& hit : hits) {
        //        hits_by_street[{ hit.street, hit.st_side }].push_back(hit);
        //    }
        //    for (auto& [st_side_key, st_hits] : hits_by_street)
        //    {
        //        auto& [street, st_side] = st_side_key;
        //        
        //            if (std::ranges::find_if(street->nodes, [](auto& n) { return n.id == 10958451055; }) != street->nodes.end())
        //            {
        //                logMESSAGE("Debug");
        //            }
        //        // sort by footpath seg param
        //        std::sort(st_hits.begin(), st_hits.end(),
        //            [](const ft_hit_info& a, const ft_hit_info& b) {
        //                if (a.ft_seg_pidx == b.ft_seg_pidx) {
        //                    return a.ft_seg_param < b.ft_seg_param;
        //                }
        //                return a.ft_seg_pidx < b.ft_seg_pidx;
        //            });
        //        st_outline outline;
        //        outline.dir = st_side;
        //        for (auto& hit : st_hits)
        //        {
        //            // get footpath point
        //            segment ft_seg = path_seg(ft_path, hit.ft_seg_pidx);
        //            glm::dvec2 ft_point = ft_seg.first + hit.ft_seg_param * (ft_seg.second - ft_seg.first);
        //            outline.nodes.push_back({
        //                .id = -1,
        //                .vert = ft_point
        //            });
        //        }
        //        if (!outline.nodes.empty())
        //        {
        //            outline.start_proj = {
        //                .pt_param = st_hits.front().st_param,
        //                .seg_pidx = st_hits.front().st_seg_pidx
        //            };
        //            outline.end_proj = {
        //                .pt_param = st_hits.back().st_param,
        //                .seg_pidx = st_hits.back().st_seg_pidx
        //            };
        //            // align outline with street direction
        //            auto& sp = outline.start_proj, & ep = outline.end_proj;
        //            if (sp.seg_pidx > ep.seg_pidx ||
        //                (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param))
        //            {
        //                std::reverse(outline.nodes.begin(), outline.nodes.end());
        //                std::swap(sp, ep);
        //            }
        //            st_outlines[street].street = street;
        //            st_outlines[street].outlines.push_back(std::move(outline));
        //        }
        //    }
        //}
        
        // convert to vector
        std::vector<st_outlines_entry> ret;
        for (auto& [street, entry] : st_outlines) {
            ret.push_back(std::move(entry));
        }
        return ret;

        {
        //struct outline_piece
        //{
        //    outline_node start, end;
        //    outline_proj::pdata start_proj, end_proj;
        //    const tree_path_seg* seg;
        //    bool joined = false;
        //};

        //std::vector<outline_piece> pieces;
        //types::unord_flat_map<const way_net::path*, types::flat_set<int>> street_piece_ids;
        //types::unord_flat_map<const tree_path_seg*, types::small_vector<int, 2>> fseg_piece_ids;

        //struct sample_data
        //{
        //    glm::dvec2 point;
        //    outline_proj proj;
        //};
        //std::vector<sample_data> samples;

        //for (auto& fseg : m_footpath_segs)
        //{
        //    glm::dvec2 fseg_vec = fseg.end.vert - fseg.start.vert;
        //    glm::dvec2 fseg_perp_dir = glm::normalize(vec_perp(fseg_vec));

        //    int num_samples = std::max(1, int(std::ceil(fseg.length / SAMPLE_INTERVAL - eps))) + 1;
        //    for (int i = 0; i < num_samples; ++i) {
        //        samples.push_back({ .point = fseg.start.vert + (double(i) / (num_samples - 1)) * fseg_vec });
        //    }

        //    //for (direction dir : { DIR_LEFT, DIR_RIGHT })
        //    //{
        //        for (int i = 0; i < num_samples; ++i) {
        //            samples[i].proj = get_street_proj(samples[i].point, &fseg, eps);
        //        }

        //        int rayidx = 0;
        //        while (rayidx < num_samples)
        //        {
        //            // get largest piece that hits the same street
        //            int pc_startidx = rayidx;
        //            while (pc_startidx < num_samples && !samples[pc_startidx].proj.street) {
        //                pc_startidx++;
        //            }
        //            int pc_endidx = pc_startidx + 1;
        //            while (pc_endidx < num_samples && samples[pc_endidx].proj.street &&
        //                samples[pc_endidx].proj.street == samples[pc_startidx].proj.street) {
        //                pc_endidx++;
        //            }

        //            if (pc_startidx < num_samples && pc_endidx - pc_startidx > 1)
        //            {
        //                outline_piece piece;
        //                piece.seg = &fseg;

        //                if (pc_startidx == 0) {
        //                    piece.start = outline_node::osm(fseg.start);
        //                } else {
        //                    piece.start.id = -1;
        //                    piece.start.vert = samples[pc_startidx].point;
        //                }
        //                if (pc_endidx == num_samples) {
        //                    piece.end = outline_node::osm(fseg.end);
        //                } else {
        //                    piece.end.id = -1;
        //                    piece.end.vert = samples[pc_endidx - 1].point;
        //                }

        //                piece.start_proj = samples[pc_startidx].proj.data;
        //                piece.end_proj = samples[pc_endidx - 1].proj.data;

        //                int piece_id = int(pieces.size());
        //                auto* hit_st = samples[pc_startidx].proj.street;

        //                pieces.push_back(piece);
        //                fseg_piece_ids[&fseg].push_back(piece_id);
        //                street_piece_ids[hit_st].insert(piece_id);
        //            }

        //            rayidx = pc_endidx;
        //        }
        //    //}

        //    // does not free capacity
        //    samples.clear();
        //}

        //using st_pieces_itr = decltype(street_piece_ids)::iterator;

        //// Join pieces by ID to build outline.
        //auto join_outline_from_pieces = [&](const st_pieces_itr& st_itr, 
        //    outline_piece* seed_piece, st_outline& outline, direction& out_start_dir, direction& out_end_dir)
        //{
        //    auto& street = st_itr->first;
        //    auto& st_piece_ids = st_itr->second;

        //    const outline_piece* start_piece, *end_piece;

        //    // go to the start of the chain
        //    auto* cur_piece = seed_piece;
        //    while (true)
        //    {
        //        int prev_segidx = cur_piece->seg->prev_seg_gidx;
        //        auto* prev_seg = prev_segidx != -1 ? &m_footpath_segs[prev_segidx] : nullptr;

        //        if (cur_piece->start.id == -1 || !prev_seg || fseg_piece_ids[prev_seg].empty()) {
        //            break;
        //        }
        //        int prev_piece_id = fseg_piece_ids[prev_seg].back();
        //        if (!st_piece_ids.contains(prev_piece_id)) {
        //            break;
        //        }
        //        auto* prev_piece = &pieces[prev_piece_id];
        //        if (prev_piece->end.id != cur_piece->start.id) {
        //            break;
        //        }
        //        cur_piece = prev_piece;
        //    }
        //    start_piece = cur_piece;

        //    // extend chain forwards to seed piece
        //    while (cur_piece != seed_piece)
        //    {
        //        cur_piece->joined = true;
        //        outline.nodes.push_back(cur_piece->start);

        //        auto* next_seg = &m_footpath_segs[cur_piece->seg->next_seg_gidx];
        //        cur_piece = &pieces[fseg_piece_ids[next_seg].front()];

        //        // every middle piece should be a complete segment
        //        assert(cur_piece == seed_piece ||
        //            (fseg_piece_ids[cur_piece->seg].size() == 1 &&
        //                cur_piece->start.id != -1 && cur_piece->end.id != -1));
        //    }

        //    // extend chain forwards
        //    while (true)
        //    {
        //        cur_piece->joined = true;
        //        outline.nodes.push_back(cur_piece->start);

        //        int next_segidx = cur_piece->seg->next_seg_gidx;
        //        auto* next_seg = next_segidx != -1 ? &m_footpath_segs[next_segidx] : nullptr;

        //        if (cur_piece->end.id == -1 || !next_seg || fseg_piece_ids[next_seg].empty()) {
        //            break;
        //        }
        //        int next_piece_id = fseg_piece_ids[next_seg].front();
        //        if (!st_piece_ids.contains(next_piece_id)) {
        //            break;
        //        }
        //        auto* next_piece = &pieces[next_piece_id];
        //        if (next_piece->start.id != cur_piece->end.id) {
        //            break;
        //        }
        //        cur_piece = next_piece;
        //    }
        //    end_piece = cur_piece;

        //    outline.nodes.push_back(cur_piece->end);
        //    assert(outline.nodes.size() >= 2);

        //    auto pt_sseg_rel_dir = [&](const glm::dvec2& pt, int segpidx) -> direction
        //    {
        //        auto seg = path_seg(street, segpidx);
        //        auto pto = orient(seg.first, seg.second, pt);
        //        switch (pto)
        //        {
        //        case ORIENT_CCW:  return DIR_LEFT;
        //        case ORIENT_COLL: return DIR_UNDEF;
        //        case ORIENT_CW:   return DIR_RIGHT;
        //        default:
        //            assert(false);
        //            return DIR_UNDEF;
        //        }
        //    };

        //    outline.start_proj = start_piece->start_proj;
        //    outline.end_proj = end_piece->end_proj;
        //    out_start_dir = pt_sseg_rel_dir(start_piece->start.vert, start_piece->start_proj.seg_pidx);
        //    out_end_dir = pt_sseg_rel_dir(end_piece->end.vert, end_piece->end_proj.seg_pidx);

        //    outline.dir = out_start_dir;
        //};

        //int num_bad_outlines = 0, num_outlines = 0;
        //
        //std::vector<st_outlines_entry> ret;
        //for (auto stpcs_itr = street_piece_ids.begin(); stpcs_itr != street_piece_ids.end(); ++stpcs_itr)
        //{
        //    assert(!stpcs_itr->second.empty());

        //    auto& entry = ret.emplace_back();
        //    entry.street = stpcs_itr->first;

        //    for (auto st_pid : stpcs_itr->second)
        //    {
        //        auto* const init_piece = &pieces[st_pid];
        //        if (init_piece->joined) {
        //            continue;
        //        }

        //        direction start_dir, end_dir;
        //        st_outline& outline = entry.outlines.emplace_back();
        //        join_outline_from_pieces(stpcs_itr, init_piece, outline, start_dir, end_dir);
        //        num_outlines++;

        //        // this means that the footpath crossed the street,
        //        // which should be impossible for real streets/sidewalks
        //        if (start_dir == DIR_UNDEF || end_dir == DIR_UNDEF || start_dir != end_dir) {
        //            entry.outlines.pop_back();
        //            num_bad_outlines++;
        //            continue;
        //        }

        //        // align outline with street direction
        //        auto& sp = outline.start_proj, &ep = outline.end_proj;
        //        if (sp.seg_pidx > ep.seg_pidx ||
        //            (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param))
        //        {
        //            std::reverse(outline.nodes.begin(), outline.nodes.end());
        //            std::swap(sp, ep);
        //        }
        //    }

        //    // if all outlines were bad, remove (very unlikely)
        //    if (entry.outlines.empty()) { 
        //        ret.pop_back();
        //    }
        //}
    
        //if (num_bad_outlines > 0) {
        //    logWARNING("Bad outlines: %d/%d", num_bad_outlines, num_outlines);
        //}
        //return ret;
        }
    }

    // Version2: single bbox/point query but compute nearest street on both sides for each sample
    //std::vector<st_outlines_entry> get_all_st_ft_outlines_v2(double eps)
    //{
    //    static constexpr double SAMPLE_INTERVAL =1; // meters
    //    constexpr double SEARCH_RADIUS =20.0; // same as param_range max in v1
    //
    //    struct outline_piece
    //    {
    //        outline_node start, end;
    //        outline_proj::pdata start_proj, end_proj;
    //        const tree_path_seg* seg;
    //        bool joined = false;
    //    };
    //
    //    std::vector<outline_piece> pieces_left;
    //    std::vector<outline_piece> pieces_right;
    //
    //    types::unord_flat_map<const way_net::path*, types::flat_set<int>> street_piece_ids_left;
    //    types::unord_flat_map<const way_net::path*, types::flat_set<int>> street_piece_ids_right;
    //    types::unord_flat_map<const tree_path_seg*, types::small_vector<int,2>> fseg_piece_ids_left;
    //    types::unord_flat_map<const tree_path_seg*, types::small_vector<int,2>> fseg_piece_ids_right;
    //
    //    for (auto& fseg : m_footpath_segs)
    //    {
    //        glm::dvec2 fseg_vec = fseg.end.vert - fseg.start.vert;
    //        glm::dvec2 fseg_perp_dir = glm::normalize(vec_perp(fseg_vec));
    //
    //        int num_samples = std::max(1, int(std::ceil(fseg.length / SAMPLE_INTERVAL - eps))) +1;
    //        std::vector<outline_proj> projs_left(num_samples, outline_proj::no_proj());
    //        std::vector<outline_proj> projs_right(num_samples, outline_proj::no_proj());
    //
    //        for (int i =0; i < num_samples; ++i) {
    //            glm::dvec2 sample = fseg.start.vert + (double(i) / (num_samples -1)) * fseg_vec;
    //
    //            // gather candidates in bbox
    //            bbox2d qb = bbox2d::empty();
    //            qb.min = sample - glm::dvec2(SEARCH_RADIUS);
    //            qb.max = sample + glm::dvec2(SEARCH_RADIUS);
    //
    //            auto cands = m_seg_tree.query_bbox_all(qb);
    //
    //            double best_left_sq = std::numeric_limits<double>::infinity();
    //            double best_right_sq = std::numeric_limits<double>::infinity();
    //            outline_proj best_left = outline_proj::no_proj();
    //            outline_proj best_right = outline_proj::no_proj();
    //
    //            for (auto cand : cands)
    //            {
    //                auto* cand_tseg = cand;
    //                if (cand_tseg->path == (&fseg)->path) continue; // skip same path
    //                if (cand_tseg->way->type != WAY_TYPE_STREET) continue;
    //
    //                // check roughly parallel
    //                double angle_bw = min_angle_bw_unitvecs(fseg.udir, cand_tseg->udir);
    //                if (std::abs(glm::degrees(angle_bw)) >45.0) continue;
    //
    //                segment seg = { cand_tseg->start.vert, cand_tseg->end.vert };
    //                glm::dvec2 ap = sample - seg.first;
    //                glm::dvec2 ab = seg.second - seg.first;
    //                double t = glm::dot(ap, ab) / glm::dot(ab, ab);
    //
    //                int seg_pidx = cand_tseg->pathidx;
    //                if ((t <0 && seg_pidx ==0) || (t >1.0 && seg_pidx == path_num_segs(cand_tseg->path) -1)) continue;
    //
    //                double t_clamped = std::clamp(t,0.0,1.0);
    //                glm::dvec2 projpt = seg.first + t_clamped * ab;
    //                double sqdist = vec_sqlength(sample - projpt);
    //
    //                if (sqdist <1e-12 || sqdist > SEARCH_RADIUS*SEARCH_RADIUS) continue;
    //
    //                // determine side relative to footpath direction
    //                double crossv = fseg.udir.x * (projpt.y - sample.y) - fseg.udir.y * (projpt.x - sample.x);
    //                if (crossv >0) {
    //                    if (sqdist < best_left_sq) {
    //                        best_left_sq = sqdist;
    //                        best_left.street = cand_tseg->path;
    //                        best_left.data = { .pt_param = t, .seg_pidx = cand_tseg->pathidx };
    //                    }
    //                } else if (crossv <0) {
    //                    if (sqdist < best_right_sq) {
    //                        best_right_sq = sqdist;
    //                        best_right.street = cand_tseg->path;
    //                        best_right.data = { .pt_param = t, .seg_pidx = cand_tseg->pathidx };
    //                    }
    //                } else {
    //                    // collinear - pick as either, choose left arbitrarily
    //                    if (sqdist < best_left_sq) {
    //                        best_left_sq = sqdist;
    //                        best_left.street = cand_tseg->path;
    //                        best_left.data = { .pt_param = t, .seg_pidx = cand_tseg->pathidx };
    //                    }
    //                }
    //            }
    //
    //            projs_left[i] = best_left;
    //            projs_right[i] = best_right;
    //        }
    //
    //        // build pieces for left side
    //        auto build_pieces_from_projs = [&](const std::vector<outline_proj>& projs, 
    //            std::vector<outline_piece>& pieces, 
    //            types::unord_flat_map<const way_net::path*, types::flat_set<int>>& street_piece_ids,
    //            types::unord_flat_map<const tree_path_seg*, types::small_vector<int,2>>& fseg_piece_ids)
    //        {
    //            int idx =0;
    //            while (idx < num_samples)
    //            {
    //                int pc_startidx = idx;
    //                while (pc_startidx < num_samples && !projs[pc_startidx].street) pc_startidx++;
    //                int pc_endidx = pc_startidx +1;
    //                while (pc_endidx < num_samples && projs[pc_endidx].street && projs[pc_endidx].street == projs[pc_startidx].street) pc_endidx++;
    //
    //                if (pc_startidx < num_samples && pc_endidx - pc_startidx >1)
    //                {
    //                    outline_piece piece;
    //                    piece.seg = &fseg;
    //
    //                    if (pc_startidx ==0) piece.start = outline_node::osm(fseg.start);
    //                    else { piece.start.id = -1; piece.start.vert = fseg.start.vert + (double(pc_startidx) / (num_samples -1)) * (fseg.end.vert - fseg.start.vert); }
    //
    //                    if (pc_endidx == num_samples) piece.end = outline_node::osm(fseg.end);
    //                    else { piece.end.id = -1; piece.end.vert = fseg.start.vert + (double(pc_endidx -1) / (num_samples -1)) * (fseg.end.vert - fseg.start.vert); }
    //
    //                    piece.start_proj = projs[pc_startidx].data;
    //                    piece.end_proj = projs[pc_endidx -1].data;
    //
    //                    int piece_id = int(pieces.size());
    //                    pieces.push_back(piece);
    //                    fseg_piece_ids[&fseg].push_back(piece_id);
    //                    street_piece_ids[projs[pc_startidx].street].insert(piece_id);
    //                }
    //
    //                idx = pc_endidx;
    //            }
    //        };
    //
    //        build_pieces_from_projs(projs_left, pieces_left, street_piece_ids_left, fseg_piece_ids_left);
    //        build_pieces_from_projs(projs_right, pieces_right, street_piece_ids_right, fseg_piece_ids_right);
    //    }
    //
    //    auto join_and_build = [&](auto& pieces, auto& fseg_piece_ids, auto& street_piece_ids)
    //    -> std::vector<st_outlines_entry>
    //    {
    //        std::vector<st_outlines_entry> ret;
    //
    //        for (auto stpcs_itr = street_piece_ids.begin(); stpcs_itr != street_piece_ids.end(); ++stpcs_itr)
    //        {
    //            assert(!stpcs_itr->second.empty());
    //
    //            auto& entry = ret.emplace_back();
    //            entry.street = stpcs_itr->first;
    //
    //            for (auto st_pid : stpcs_itr->second)
    //            {
    //                auto* const init_piece = &pieces[st_pid];
    //                if (init_piece->joined) continue;
    //
    //                // join similar to v1
    //                auto* cur_piece = init_piece;
    //                while (true)
    //                {
    //                    int prev_segidx = cur_piece->seg->prev_seg_gidx;
    //                    auto* prev_seg = prev_segidx != -1 ? &m_footpath_segs[prev_segidx] : nullptr;
    //                    if (cur_piece->start.id == -1 || !prev_seg || fseg_piece_ids[prev_seg].empty()) break;
    //                    int prev_piece_id = fseg_piece_ids[prev_seg].back();
    //                    if (!stpcs_itr->second.contains(prev_piece_id)) break;
    //                    auto* prev_piece = &pieces[prev_piece_id];
    //                    if (prev_piece->end.id != cur_piece->start.id) break;
    //                    cur_piece = prev_piece;
    //                }
    //                auto* start_piece = cur_piece;
    //
    //                // forward to seed
    //                while (cur_piece != init_piece)
    //                {
    //                    cur_piece->joined = true;
    //                    // push start
    //                    // we'll collect into outline
    //                    auto* next_seg = &m_footpath_segs[cur_piece->seg->next_seg_gidx];
    //                    cur_piece = &pieces[fseg_piece_ids[next_seg].front()];
    //                }
    //
    //                // now extend forward from init_piece
    //                st_outline outline;
    //                cur_piece = init_piece;
    //                while (true)
    //                {
    //                    cur_piece->joined = true;
    //                    outline.nodes.push_back(cur_piece->start);
    //
    //                    int next_segidx = cur_piece->seg->next_seg_gidx;
    //                    auto* next_seg = next_segidx != -1 ? &m_footpath_segs[next_segidx] : nullptr;
    //
    //                    if (cur_piece->end.id == -1 || !next_seg || fseg_piece_ids[next_seg].empty()) break;
    //                    int next_piece_id = fseg_piece_ids[next_seg].front();
    //                    if (!stpcs_itr->second.contains(next_piece_id)) break;
    //                    auto* next_piece = &pieces[next_piece_id];
    //                    if (next_piece->start.id != cur_piece->end.id) break;
    //                    cur_piece = next_piece;
    //                }
    //
    //                outline.nodes.push_back(cur_piece->end);
    //                outline.start_proj = start_piece->start_proj;
    //                outline.end_proj = cur_piece->end_proj;
    //
    //                // compute dirs
    //                auto pt_sseg_rel_dir = [&](const glm::dvec2& pt, int segpidx) -> direction
    //                {
    //                    auto seg = path_seg(entry.street, segpidx);
    //                    auto pto = orient(seg.first, seg.second, pt);
    //                    switch (pto)
    //                    {
    //                    case ORIENT_CCW: return DIR_LEFT;
    //                    case ORIENT_COLL: return DIR_UNDEF;
    //                    case ORIENT_CW: return DIR_RIGHT;
    //                    default:
    //                        assert(false);
    //                        return DIR_UNDEF;
    //                    }
    //                };
    //
    //                direction sd = pt_sseg_rel_dir(outline.nodes.front().vert, outline.start_proj.seg_pidx);
    //                direction ed = pt_sseg_rel_dir(outline.nodes.back().vert, outline.end_proj.seg_pidx);
    //
    //                if (sd == DIR_UNDEF || ed == DIR_UNDEF || sd != ed) {
    //                    continue;
    //                }
    //
    //                outline.dir = sd;
    //
    //                // ensure order
    //                auto& sp = outline.start_proj; auto& ep = outline.end_proj;
    //                if (sp.seg_pidx > ep.seg_pidx || (sp.seg_pidx == ep.seg_pidx && sp.pt_param > ep.pt_param)) {
    //                    std::reverse(outline.nodes.begin(), outline.nodes.end());
    //                    std::swap(sp, ep);
    //                }
    //
    //                entry.outlines.push_back(std::move(outline));
    //            }
    //
    //            if (entry.outlines.empty()) ret.pop_back();
    //        }
    //
    //        return ret;
    //    };
    //
    //    auto left_entries = join_and_build(pieces_left, fseg_piece_ids_left, street_piece_ids_left);
    //    auto right_entries = join_and_build(pieces_right, fseg_piece_ids_right, street_piece_ids_right);
    //
    //    // merge left and right entries into single vector grouping by street
    //    types::unord_flat_map<const way_net::path*, st_outlines_entry> merged_map;
    //    for (auto& e : left_entries) {
    //        merged_map[e.street].street = e.street;
    //        for (auto& o : e.outlines) merged_map[e.street].outlines.push_back(std::move(o));
    //    }
    //    for (auto& e : right_entries) {
    //        merged_map[e.street].street = e.street;
    //        for (auto& o : e.outlines) merged_map[e.street].outlines.push_back(std::move(o));
    //    }
    //
    //    std::vector<st_outlines_entry> ret;
    //    for (auto& kv : merged_map) ret.push_back(std::move(kv.second));
    //    return ret;
    //}

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
    street_outlines_t build_outlines(double eps, int strategy = 1)
    {
        auto tbegin_outlines = clk::now();
        std::vector<st_outlines_entry> entries = get_all_st_ft_outlines(eps);
        //if (strategy == 1) {
        //    entries = get_all_st_ft_outlines(eps);
        //} else if (strategy == 2) {
        //    entries = get_all_st_ft_outlines_v2(eps);
        //} else {
        //    // default to v1
        //    entries = get_all_st_ft_outlines(eps);
        //}
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
    const way_net& m_network;
    const way_net_paths& m_all_paths;
    std::vector<tree_path_seg> m_footpath_segs;
    std::vector<tree_path_seg> m_street_segs;
    aabb_tree2d<tree_path_seg*> m_seg_tree;
    const aabb_tree2d<mesh_builder::building*>& m_bldg_tree;
    std::function<void(const char*, clk::duration)> m_step_done;
};


static void gen_path_drawdata(draw_datad& dd, const way_net::path& path, double eps)
{
    std::vector<glm::dvec2> verts(path.nodes.size());
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        verts[i] = path.nodes[i].vert();
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
    }
    step_done("Street network", clk::now() - tbegin_net);
    
    auto tbegin_paths_ex = clk::now();
    way_net_paths all_paths;
    {
        all_paths = get_all_paths(network);
    }
    step_done("Path extraction", clk::now() - tbegin_paths_ex);

    st_outline_builder st_builder(network, all_paths, *bldg_tree_ptr, step_done);
    auto st_outline_map = st_builder.build_outlines(eps, 1);
    //auto st_outline_map = st_outline_builder::street_outlines_t{};

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

    auto tbegin_outlines = clk::now();
    {
        for (auto& [street, outline] : st_outline_map) {
            if (!gen_outline_drawdata(street_dd, outline)) {
                gen_path_drawdata(street_dd, *street, eps);
            }
        }
    }
    step_done("Outline triangulation", clk::now() - tbegin_outlines);

    auto tbegin_paths = clk::now();
    {
        for (const auto& footpath : all_paths.footpaths) {
            gen_path_drawdata(footpath_dd, footpath, eps);
        }
        for (const auto& street : all_paths.streets) {
            if (!st_outline_map.contains(&street)) {
                gen_path_drawdata(street_dd, street, eps);
            }
        }
    }
    step_done("Path triangulation", clk::now() - tbegin_paths);

    uint32_t num_tris = street_dd.num_tris() + footpath_dd.num_tris();
    uint32_t num_verts = street_dd.num_verts() + footpath_dd.num_verts();

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));
    //drawdata.push_back(std::move(debug_dd));

    auto tend = clk::now();

    logMESSAGE("Generated %u tris and %u vertices in %s", 
        num_tris, num_verts, time_str(tend - tbegin).c_str());

    return true;
}
