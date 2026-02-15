
#include "common.hpp"
#include "../containers/way_network.hpp"
#include "../containers/aabb_tree.hpp"

#include "st_ft_outline_builder.hpp"

// Strategy:
// Snap the street segments to the footpaths, if they are within a certain distance and angular tolerance
// Consider left and right outlines of the street separately.
// If footpath is not present, just draw the outline normally, by using estimated width from OSM tags.
// Smoothly interpolate between street segments with different widths.

//todo: I can start cleaning up now and reimplementing parts of the code
// some footpaths are inside area-mapped highways, so I need to check if the footpath is inside an area and remove it, just generate the area
// some footpaths are at different heights than the street! These should be ignored or drawn appropriately
// some footpaths are obscured by buildings

class st_outline_builder
{
public:
    st_outline_builder(const way_net& network, const way_net_paths& all_paths,
        const aabb_tree2d<mesh_builder::building*>& bldg_tree) :
        m_network(network),
        m_all_paths(all_paths), 
        m_bldg_tree(bldg_tree)
    {
        timeit("Outline extraction prep", [&]()
        {
            auto ftseg_bnds = add_path_segments(all_paths.footpaths, m_all_segs);
            auto stseg_bnds = add_path_segments(all_paths.streets, m_all_segs);

            m_footpath_segs = std::span(m_all_segs).subspan(ftseg_bnds.first, ftseg_bnds.second);
            m_street_segs = std::span(m_all_segs).subspan(stseg_bnds.first, stseg_bnds.second);

            size_t tobj_idx = 0;
            buffer<path_tseg*> tree_objects(int(m_all_segs.size()), buffer_overwrite);
            for (auto& seg : m_all_segs) {
                tree_objects.ptr[tobj_idx++] = &seg;
            }
            m_seg_tree = aabb_tree2d<path_tseg*>::create_unsafe(tree_objects.span());
        });
    }

private:
    struct path_tseg
    {
        int pidx; // Index in path
        double length;
        glm::dvec2 udir; // Unit direction
        bbox2d bbox;

        const way_net::path* path;
        const mesh_builder::highway* way;

        const osm_node& start() const { 
            return path->nodes[pidx].osm_node; 
        }
        const osm_node& end() const { 
            return path->nodes[pidx + 1].osm_node;
        }

        struct aabb_traits {
            static const bbox2d& bbox(const path_tseg* seg) {
                return seg->bbox;
            }
        };
    };

    std::pair<int, int> add_path_segments(
        const std::vector<way_net::path>& paths, std::vector<path_tseg>& segments)
    {
        int start_idx = int(segments.size());
        for (auto& path : paths)
        {
            assert_msg(path.nodes.size() >= 2, "bad path");

            m_path_segs_startgidx[&path] = int(segments.size());

            for (int i = 0; i < path_num_segs(&path); ++i)
            {
                auto& start = path.nodes[i];
                auto& end = path.nodes[i + 1];

                auto bbox = bbox2d::empty();
                bbox.extend(start.vert());
                bbox.extend(end.vert());

                double length = glm::length(end.vert() - start.vert());

                segments.push_back({
                    .pidx = i,
                    .length = length,
                    .udir = (end.vert() - start.vert()) / length,
                    .bbox = bbox,
                    .path = &path,
                    .way = end.in_way
                });
            }
        }
        return { start_idx, int(segments.size()) - start_idx };
    }

    enum st_param_type
    {
        ST_PARAM_SEGMENT = 0,
        ST_PARAM_JUNCTION = 1
    };

    struct st_param
    {
        // Segment is at seg_pidx
        // Junction is b/w seg_pidx and seg_pidx + 1
        int seg_pidx;
        st_param_type type;
        // Dist param on segment, or angle param at junction
        double param;
        // Junction only, rotation applied to segment normal.
        // +ve is CCW, -ve is CW.
        double junc_rot;

        static st_param st_begin() {
            return { 
                .seg_pidx = 0, 
                .type = ST_PARAM_SEGMENT, 
                .param = 0.0 
            };
        }
        static st_param st_end(const way_net::path* street) {
            return { 
                .seg_pidx = path_num_segs(street) - 1, 
                .type = ST_PARAM_SEGMENT, 
                .param = 1.0,
                .junc_rot = 0.0
            };
        }

        bool operator<(const st_param& other) const 
        {
            if (seg_pidx != other.seg_pidx) { return seg_pidx < other.seg_pidx; }
            if (type != other.type) { return type < other.type; }
            return param < other.param;
        }

        bool operator==(const st_param& other) const {
            return seg_pidx == other.seg_pidx && type == other.type && param == other.param;
        }

        bool operator>(const st_param& other) const {
            return other < *this;
        }
    };
    
    static constexpr double ST_FT_MAX_DIST = 25.0;
    static constexpr double ST_FT_MAX_ANGLE = glm::radians(45.0);
    
    static constexpr double ST_RAYCAST_STEP = 1; // meters
    static constexpr double ST_JUNC_ANGLE_STEP = ST_RAYCAST_STEP / ST_FT_MAX_DIST; // radians

    struct st2ft_hit
    {
        const way_net::path* footpath;
        double seg_param;
        int seg_pidx;
    };

    std::optional<st2ft_hit> st2ft_ray(const path_tseg& sseg, const ray2d& ray, st_param_type type, double eps)
    {
        struct tree_qdata
        {
            double sqdist;
            double pt_param;
        };
        tree_qdata hit_segdata{};
        path_tseg* hit_seg = nullptr;
        auto dist_range = param_range{ 0.0, ST_FT_MAX_DIST };

        bool hit = m_seg_tree.query_nearest(ray, dist_range,
            [&](path_tseg* cand_tseg, tree_qdata& out_qdata) -> bool
            {
                if (cand_tseg == &sseg || cand_tseg->way->layer != sseg.way->layer) {
                    return false;
                }
                // check if somewhat parallel
                if (type == ST_PARAM_SEGMENT && cand_tseg->way->type == WAY_TYPE_FOOTWAY) {
                    double angle_bw = acute_angle_bw_unitvecs(sseg.udir, cand_tseg->udir);
                    if (angle_bw > ST_FT_MAX_ANGLE) {
                        return false;
                    }
                }

                segment ray_ptseg = { ray.at_param(dist_range.min()), ray.at_param(dist_range.max()) };
                segment cand_ptseg = { cand_tseg->start().vert, cand_tseg->end().vert};
                auto inter_res = seg_intersect(ray_ptseg, cand_ptseg, eps);

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
                .seg_pidx = hit_seg->pidx
            };
        }
        return std::nullopt;
    }

    struct st_outline_frag
    {
        std::vector<outline_node> nodes;
        st_param start_stpm, end_stpm;
        direction side;
    };
    using st_outline_frags_t = types::unord_flat_map<const way_net::path*, std::vector<st_outline_frag>>;

    // Collect street outline fragments from nearby footpaths
    st_outline_frags_t get_outline_fragments(double eps)
    {
        struct ft_hit_info
        {
            const way_net::path* street;
            direction st_side;
            st_param_type type;
            int ft_seg_pidx;
            int st_seg_pidx;
            double ft_seg_param;
            double st_param;
            double st_junc_rot;

            st_outline_builder::st_param get_st_param() const {
                return { 
                    .seg_pidx = st_seg_pidx, 
                    .type = type, 
                    .param = st_param,
                    .junc_rot = st_junc_rot
                };
            }
        };

        types::unord_flat_map<const way_net::path*, std::vector<ft_hit_info>> ft_hits;

        auto do_st2ft_ray = [&](const ray2d& ray, st_param_type type, 
            const path_tseg& sseg, direction st_side, double st_param, double st_junc_rot = 0.0)
        {
            auto hit = st2ft_ray(sseg, ray, type, eps);
            if (hit.has_value())
            {
                ft_hits[hit->footpath].push_back({
                    .street = sseg.path,
                    .st_side = st_side,
                    .type = type,
                    .ft_seg_pidx = hit->seg_pidx,
                    .st_seg_pidx = sseg.pidx,
                    .ft_seg_param = hit->seg_param,
                    .st_param = st_param,
                    .st_junc_rot = st_junc_rot
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

                        do_st2ft_ray(ray, ST_PARAM_SEGMENT, sseg, st_side, t);
                    }
                }

                // cover corners with large angles
                if (sseg_idx != street_num_segs - 1) 
                {
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
                    int junc_num_rays = std::max(0, int(std::ceil(sweep_angle / ST_JUNC_ANGLE_STEP)) - 1);
                    if (junc_num_rays == 0) {
                        continue; // straight line
                    }

                    for (int i = 1; i <= junc_num_rays; ++i) 
                    {
                        double t = double(i) / (junc_num_rays + 1);
                        double rot = t * sweep_angle * int(sweep_orient);
                        ray2d ray{ .origin = sseg.end().vert, .dir = rotate_vec2(sweep_start, rot)};

                        do_st2ft_ray(ray, ST_PARAM_JUNCTION, sseg, st_side, t, rot);
                    }
                }
            }
        }

        st_outline_frags_t ret;

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

            using T = std::pair<decltype(left_hits)&, direction>;

            for (auto& data : { T{ left_hits, DIR_LEFT }, T{ right_hits, DIR_RIGHT } })
            {
                auto& hits = data.first;
                direction st_side = data.second;
                          
                std::sort(hits.begin(), hits.end(), [](auto* a, auto* b) {
                    if (a->ft_seg_pidx != b->ft_seg_pidx) {
                        return a->ft_seg_pidx < b->ft_seg_pidx;
                    }
                    return a->ft_seg_param < b->ft_seg_param;
                });

                struct st_and_frag
                {
                    const way_net::path* street;
                    std::vector<outline_node> nodes;
                    const ft_hit_info* start_hit, *end_hit;
                };

                auto get_ft_node = [&](int idx) -> outline_node {
                    auto& n = ft_path->nodes[idx];
                    return outline_node::osm(n.osm_node);
                };

                auto get_endpoint = [&](const ft_hit_info* hitp) -> outline_node
                {
                    if (hitp->ft_seg_param < eps) {
                        return get_ft_node(hitp->ft_seg_pidx);
                    }
                    else if (hitp->ft_seg_param > 1.0 - eps) {
                        return get_ft_node(hitp->ft_seg_pidx + 1);
                    }
                    else {
                        auto pt = path_point(ft_path, hitp->ft_seg_pidx, hitp->ft_seg_param);
                        return { .id = -1, .vert = pt };
                    }
                };

                std::vector<st_and_frag> frags;
                for (auto& hitp : hits)
                {
                    if (frags.empty() || frags.back().street != hitp->street) {
                        frags.push_back({
                            .street = hitp->street,
                            .start_hit = hitp,
                            .end_hit = nullptr
                        });
                    }
                    auto& frag = frags.back();
                    if (frag.end_hit) 
                    {
                        // intermediate nodes only, last node added later
                        const int st_nidx = frag.end_hit->ft_seg_pidx + 1;
                        for (int nidx = st_nidx; nidx <= hitp->ft_seg_pidx; ++nidx) {
                            frag.nodes.push_back(get_ft_node(nidx));
                        }
                    }
                    else { frag.nodes.push_back(get_endpoint(hitp)); } // first node

                    frag.end_hit = hitp;
                }

                for (auto& frag : frags)
                {
                    // last node
                    auto last_point = get_endpoint(frag.end_hit);
                    if (last_point.id == -1 || last_point.id != frag.nodes.back().id) {
                        frag.nodes.push_back(last_point);
                    }

                    st_outline_frag ret_frag = {
                        .nodes = std::move(frag.nodes),
                        .start_stpm = frag.start_hit->get_st_param(),
                        .end_stpm = frag.end_hit->get_st_param(),
                        .side = st_side
                    };

                    // align with street
                    auto& spm = ret_frag.start_stpm, &epm = ret_frag.end_stpm;
                    if (spm > epm) {
                        std::reverse(ret_frag.nodes.begin(), ret_frag.nodes.end());
                        std::swap(spm, epm);
                    }

                    ret[frag.street].push_back(std::move(ret_frag));
                }
            }
        }

        return ret;
    }

    std::vector<outline_node> join_fragments_for_side(const way_net::path* street,
        std::span<const path_tseg> street_tsegs, std::vector<const st_outline_frag*>& fragments, direction side, double eps)
    {
        assert(side == DIR_LEFT || side == DIR_RIGHT);

        std::vector<outline_node> ret;
        auto add_genpoint = [&](const glm::dvec2& vert) {
            ret.push_back({ .id = -1, .vert = vert });
        };

        auto seg_point = [&](const st_param& stpm) -> glm::dvec2 
        {
            assert(stpm.type == ST_PARAM_SEGMENT);
            return path_point(street, stpm.seg_pidx, stpm.param);
        };

        auto seg_widthvec = [&](int seg_pidx) -> glm::dvec2
        {
            auto& seg = street_tsegs[seg_pidx];
            double width = path_seg_width(street, seg_pidx);
            return (side == DIR_LEFT ? 1 : -1) * (width / 2.0) * vec_perp(seg.udir);
        };

        auto junc_point = [&](const st_param& stpm) -> glm::dvec2
        {
            assert(stpm.type == ST_PARAM_JUNCTION);
            return street->nodes[stpm.seg_pidx + 1].vert();
        };

        auto junc_widthvec = [&](const st_param& stpm) -> glm::dvec2
        {
            assert(stpm.type == ST_PARAM_JUNCTION);

            double w1 = path_seg_width(street, stpm.seg_pidx);
            double w2 = path_seg_width(street, stpm.seg_pidx + 1);
            double width = (1.0 - stpm.param) * w1 + stpm.param * w2;
            
            glm::dvec2 init_vec = (side == DIR_LEFT ? 1.0 : -1.0) * 
                (width / 2.0) * vec_perp(street_tsegs[stpm.seg_pidx].udir);
            
            return rotate_vec2(init_vec, stpm.junc_rot);
        };

        auto fill_seg_to_seg_hole = [&](const st_param& start_pm, const st_param& end_pm)
        {
            assert(start_pm.type == ST_PARAM_SEGMENT && end_pm.type == ST_PARAM_SEGMENT);

            if (start_pm.seg_pidx < end_pm.seg_pidx)
            {
                glm::dvec2 start_pt = seg_point(start_pm) + seg_widthvec(start_pm.seg_pidx);
                glm::dvec2 end_pt = seg_point(end_pm) + seg_widthvec(end_pm.seg_pidx);

                add_genpoint(start_pt);
                for (int seg_pidx = start_pm.seg_pidx; seg_pidx < end_pm.seg_pidx; ++seg_pidx)
                {
                    auto& cur_seg = street_tsegs[seg_pidx];
                    auto& next_seg = street_tsegs[seg_pidx + 1];
                    double width_diff = path_seg_width(street, seg_pidx) - path_seg_width(street, seg_pidx + 1);

                    if (angle_bw_unitvecs(cur_seg.udir, next_seg.udir) > glm::radians(2.0) || std::abs(width_diff) > eps) {
                        add_genpoint(cur_seg.end().vert + seg_widthvec(seg_pidx));
                        add_genpoint(cur_seg.end().vert + seg_widthvec(seg_pidx + 1));
                    }
                }
                add_genpoint(end_pt);
            }
            else if (start_pm.seg_pidx == end_pm.seg_pidx && start_pm.param < end_pm.param)
            {
                glm::dvec2 norm = seg_widthvec(start_pm.seg_pidx);
                add_genpoint(path_point(street, start_pm.seg_pidx, start_pm.param) + norm);
                add_genpoint(path_point(street, start_pm.seg_pidx, end_pm.param) + norm);
            }
        };

        auto fill_junc_hole = [&](const st_param& junc_pm, double end_param, bool keep_firstpt, bool keep_lastpt)
        {
            assert(junc_pm.type == ST_PARAM_JUNCTION);

            double angle_bw_segs = junc_pm.junc_rot / junc_pm.param; // oriented
            
            const double start_rot = junc_pm.junc_rot;
            const double end_rot = end_param * angle_bw_segs;
            const double rot_diff = end_rot - start_rot;

            constexpr double angle_step = 10.0;
            constexpr int max_steps = int(180.0 / angle_step) + 2;
            int num_steps = std::max(0, int(std::ceil(std::abs(rot_diff) / glm::radians(angle_step)) - 1));
            
            int pt_buf_idx = 0;
            glm::dvec2 pt_buf[max_steps];
            
            auto junc_pt = junc_point(junc_pm);

            if (keep_firstpt) {
                pt_buf[pt_buf_idx++] = junc_pt + junc_widthvec(junc_pm);
            }
            for (int i = 1; i <= num_steps; ++i) 
            {
                double inter_rot = start_rot + (double(i) / (num_steps + 1)) * rot_diff;
                st_param inter_pm = {
                    .seg_pidx = junc_pm.seg_pidx,
                    .type = ST_PARAM_JUNCTION,
                    .param = inter_rot / angle_bw_segs,
                    .junc_rot = inter_rot
                };
                pt_buf[pt_buf_idx++] = junc_pt + junc_widthvec(inter_pm);
            }

            if (keep_lastpt) {
                st_param end_pm = {
                    .seg_pidx = junc_pm.seg_pidx,
                    .type = ST_PARAM_JUNCTION,
                    .param = end_param,
                    .junc_rot = end_rot
                };
                pt_buf[pt_buf_idx++] = junc_pt + junc_widthvec(end_pm);
            }

            if (end_param < junc_pm.param) {
                std::reverse(pt_buf, pt_buf + pt_buf_idx);
            }
            for (int i = 0; i < pt_buf_idx; ++i) {
                add_genpoint(pt_buf[i]);
            }
        };

        std::ranges::sort(fragments, std::less<st_param>{}, &st_outline_frag::start_stpm);

        auto sthole_start = st_param::st_begin();
        for (int frag_idx = 0; frag_idx <= fragments.size(); ++frag_idx) // <= is on purpose
        {
            bool valid_frag = frag_idx < fragments.size();
            st_param sthole_end = valid_frag ? fragments[frag_idx]->start_stpm : st_param::st_end(street);

            if (sthole_start > sthole_end) {
                return {}; // todo: figure out why this happens
            }
            
            if (sthole_start < sthole_end)
            {
                if (sthole_start.type == ST_PARAM_SEGMENT && sthole_end.type == ST_PARAM_SEGMENT) {
                    fill_seg_to_seg_hole(sthole_start, sthole_end);
                }
                else if (sthole_start.type == ST_PARAM_JUNCTION && sthole_end.type == ST_PARAM_SEGMENT) 
                {
                    fill_junc_hole(sthole_start, 1.0, true, false);
                    st_param seg_hole_start = { 
                        .seg_pidx = sthole_start.seg_pidx + 1, 
                        .type = ST_PARAM_SEGMENT, 
                        .param = 0.0 
                    };
                    fill_seg_to_seg_hole(seg_hole_start, sthole_end);
                }
                else if (sthole_start.type == ST_PARAM_SEGMENT && sthole_end.type == ST_PARAM_JUNCTION) 
                {
                    st_param seg_hole_end = { 
                        .seg_pidx = sthole_end.seg_pidx, 
                        .type = ST_PARAM_SEGMENT, 
                        .param = 1.0 
                    };
                    fill_seg_to_seg_hole(sthole_start, seg_hole_end);
                    fill_junc_hole(sthole_end, 0.0, true, false);
                }
                else if (sthole_start.type == ST_PARAM_JUNCTION && sthole_end.type == ST_PARAM_JUNCTION) 
                {
                    if (sthole_start.seg_pidx == sthole_end.seg_pidx) {
                        fill_junc_hole(sthole_start, sthole_end.param, true, true);
                    } 
                    else {
                        fill_junc_hole(sthole_start, 1.0, true, false);
                        st_param seg_hole_start = { 
                            .seg_pidx = sthole_start.seg_pidx + 1, 
                            .type = ST_PARAM_SEGMENT, 
                            .param = 0.0 
                        };
                        st_param seg_hole_end = { 
                            .seg_pidx = sthole_end.seg_pidx, 
                            .type = ST_PARAM_SEGMENT, 
                            .param = 1.0 
                        };
                        fill_seg_to_seg_hole(seg_hole_start, seg_hole_end);
                        fill_junc_hole(sthole_end, 0.0, true, false);
                    }
                }
            }

            if (valid_frag) {
                for (const auto& node : fragments[frag_idx]->nodes) {
                    ret.push_back(node);
                }
                sthole_start = fragments[frag_idx]->end_stpm;
            }
        } 

        return ret;
    }

    street_outlines_t join_outline_fragments(const st_outline_frags_t& st_frags, double eps)
    {
        street_outlines_t ret;
        for (auto& [street, outlines] : st_frags)
        {
            assert(!outlines.empty());

            std::vector<const st_outline_frag*> left_frags, right_frags;
            for (auto& outline : outlines)
            {
                switch (outline.side)
                {
                case DIR_LEFT:  left_frags.push_back(&outline); break;
                case DIR_RIGHT: right_frags.push_back(&outline); break;
                default: assert(false); break;
                }
            }

            const int segs_startgidx = m_path_segs_startgidx[street];
            auto street_segs = std::span(m_all_segs).subspan(segs_startgidx, path_num_segs(street));

            auto lt_outline = join_fragments_for_side(street, street_segs, left_frags, DIR_LEFT, eps);
            auto rt_outline = join_fragments_for_side(street, street_segs, right_frags, DIR_RIGHT, eps);

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
        st_outline_frags_t fragments;
        timeit("Outline extraction", [&]() {
            fragments = get_outline_fragments(eps);
        });

        street_outlines_t ret;
        timeit("Outline joining", [&]() {
            ret = join_outline_fragments(fragments, eps);
        });

        return ret;
    }

private:
    const way_net& m_network;
    const way_net_paths& m_all_paths;
    std::vector<path_tseg> m_all_segs;
    std::span<path_tseg> m_footpath_segs;
    std::span<path_tseg> m_street_segs;
    types::unord_flat_map<const way_net::path*, int> m_path_segs_startgidx;
    aabb_tree2d<path_tseg*> m_seg_tree;
    const aabb_bldg_tree& m_bldg_tree;
};

street_outlines_t build_st_ft_outlines(const way_net& network,
    const way_net_paths& all_paths, const aabb_bldg_tree& bldg_tree, double eps)
{
    st_outline_builder builder(network, all_paths, bldg_tree);
    return builder.build_outlines(eps);
}