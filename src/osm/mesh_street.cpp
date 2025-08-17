
#include <algorithm>
#include <span>
#include <unordered_map>
#include <iterator>

#include <osmium/osm/node_ref_list.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <boost/unordered/unordered_flat_map.hpp>

#include "containers/way_network.hpp"
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
    std::vector<node_ref> nodes;
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

template <>
struct way_network_traits<mesh_builder::highway>
{
    static way_type way_type(const mesh_builder::highway* way) {
        return way->type;
    }
};

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
    node_ref start, end;
    bbox2d bbox;
    const mesh_builder::highway* way;

    // This can be used to join the outlines at the end
    const way_net::path* parent_path;

    // need the prev and next segments in the path
    // endid of the segment is the startid of the _next_ segment
    // startid of the segment is the endid of the _previous_ segment
    // this can be used to determine correct direction when joining pieces
    // nasty, replace with an index instead of pointers
    const path_seg* prev_seg;
    const path_seg* next_seg;
};

// A piece of the segment that can 
struct path_segpiece
{
    // instead of start and end of the segment, I now have several
    // pieces per path segment. I need to join them together
    // somehow to create the final joined outline. How do I deal with holes?
    // Forget holes for now, assume I get a continuous set of pieces per path segment
    // and map the streets to the pieces.
    // I need to join the pieces somehow. Joining only happens at the start and end of the segment.
    // If I use an ID instead of start and end, I can use the ID to join the pieces together.
    // I have to ensure that pieces at segment endpoints have the same start and end IDs if the segments are adjacent.

    glm::dvec2 start, end;
    // If not -1, piece starts or ends at segment node with this id
    osmium::object_id_type startid, endid;
    const path_seg* parent_seg;

    const way_net::path* hit_path; // remove?
};

struct outline_entry
{

    std::vector<path_segpiece> pieces;
};

template <>
struct aabb_tree_traits<path_seg*>
{
    static const bbox2d& bbox(const path_seg* seg) {
        return seg->bbox;
    }
};

static std::vector<std::vector<path_seg>> get_path_segments(const std::vector<way_net::path>& paths)
{
    std::vector<std::vector<path_seg>> ret;

    for (auto& path : paths)
    {
        assert_msg(path.nodes.size() >= 2, "bad path");

        std::vector<path_seg> segments;
        for (size_t i = 0; i < path.nodes.size() - 1; ++i)
        {
            auto& start = path.nodes[i];
            auto& end = path.nodes[i + 1];

            bbox2d bbox;
            bbox.extend(start.vert);
            bbox.extend(end.vert);

            segments.push_back({
                .start = start.node_ref(),
                .end = end.node_ref(),
                .bbox = bbox,
                .way = end.in_way,
                .parent_path = &path
            });
        }

        for (size_t i = 0; i < segments.size(); ++i)
        {
            segments[i].prev_seg = i > 0 ? &segments[i - 1] : nullptr;
            segments[i].next_seg = i < segments.size() - 1 ? &segments[i + 1] : nullptr;
        }
        ret.push_back(std::move(segments));
    }

    return ret;
}

struct ray_hit
{
    double dist;
    path_seg* seg;
};

static bool ray_street_hit(const ray2d& ray, const aabb_tree<path_seg*>& seg_tree, 
    const aabb_tree<mesh_builder::building*>& bldg_tree, ray_hit& out_hit, double eps)
{
    auto seg_hit_cb = [](const ray2d& ray, path_seg* cand, 
        double& out_canddist, param_range dist_range, double eps) -> bool
    {
        if (cand->way->type != WAY_TYPE_STREET) {
            return false;
        }

        // check if footpath and street are somewhat parallel
        double angle_bw = min_angle_between(ray.dir, cand->end.vert - cand->start.vert);
        if (std::abs(glm::degrees(angle_bw) - 90.0) > 45.0) {
            return false;
        }

        segment ray_seg = { ray.at_param(dist_range.min), ray.at_param(dist_range.max) };
        segment cand_seg = { cand->start.vert, cand->end.vert };

        seg_inter_result inter_res;
        if (segments_intersect(ray_seg, cand_seg, inter_res, eps)) {
            out_canddist = glm::length(inter_res.point - ray.origin);
            return true;
        }
        else { return false; }
    };

    if (seg_tree.ray_first_hit(ray, seg_hit_cb,
        out_hit.dist, out_hit.seg, { .min = 0.0, .max = 25.0 }, eps)) {
        return true;
    }
    else {
        out_hit.seg = nullptr;
        out_hit.dist = std::numeric_limits<double>::infinity();
        return false;
    }
};


static types::unord_flat_map<const way_net::path*, std::vector<std::pair<osmium::object_id_type, glm::dvec2>>>  gen_street_outlines(
    const std::vector<way_net::path>& footpaths, 
    const std::vector<way_net::path>& streets, 
    const aabb_tree<mesh_builder::building*>& bldg_tree, 
    double eps)
{
    // for each footpath segment, find nearby street segments
    // if within a certain distance and angular tolerance, associate the footpath segment to the street segment
    // one footpath segment may be associated to multiple street segments
    // one street segment may have multiple associated footpath segments
    // store these associations in a map or similar structure for later use when drawing the streets

        // traverse through all footpaths, shoot rays to nearby streets
    // and mark those as streets as near
    // 

    // on cutting footpaths to match nearby streets:
    // go through all footpaths above. Each footpath can be cut into multiple footpath pieces
    // The cut is made when the distance or and angular tolerance to nearby street is broken
    // or more precisely, when the matched street segment changes
    // This allows cutting footpaths to match their corresponding street segments

    auto footpath_segments = get_path_segments(footpaths);
    auto street_segments = get_path_segments(streets);

    aabb_tree<path_seg*> seg_tree;
    {
        std::vector<path_seg*> tree_objects;
        for (auto* p : { &footpath_segments, &street_segments }) {
            for (auto& path_segs : *p) {
                for (auto& seg : path_segs) {
                    tree_objects.push_back(&seg);
                }
            }
        }
        seg_tree = aabb_tree<path_seg*>::create_unsafe(tree_objects);
    }

    static constexpr double RAY_FIRE_INTERVAL = 5.0; // meters

    std::pmr::pool_options opts;
    opts.max_blocks_per_chunk = 2;
    opts.largest_required_pool_block = 0;

    std::pmr::unsynchronized_pool_resource pool{ opts };
    std::pmr::polymorphic_allocator<ray_hit> rayhit_alloc { &pool };
    std::pmr::polymorphic_allocator<glm::dvec2> rayorig_alloc { &pool };

    // shoot a ray in the perpendicular direction
            // if in the same direction, the street hit changes as I sweep through the footpath segment,
            // cut the footpath segment at that point. The cut cannot be done here since the aabb_tree
            // would be invalidated, so the cut info needs to be stored 
            // maybe I don't need to cut the footpath segment, but rather just add additional points
            // in between that can be used to draw the street outlines, that way no additional footpath segs need to be added
            // and the footpath segment can be used as is

            // street_outlines can be a map of street paths to a vector of footpath segment pieces (pieces are generated from cuts)

    types::unord_flat_map<const way_net::path*, std::vector<std::pair<osmium::object_id_type, glm::dvec2>>> street_outlines;
    types::unord_flat_map<const way_net::path*, std::vector<path_segpiece>> street_outline_pieces;

    auto add_outline_pieces = [&](path_seg& seg, std::span<const glm::dvec2> ray_origins, glm::dvec2 ray_dir)
    {
        size_t num_rays = ray_origins.size();
        auto ray_hits = rayhit_alloc.allocate(num_rays);

        for (size_t i = 0; i < num_rays; ++i)
        {
            ray2d ray{ .origin = ray_origins[i], .dir = ray_dir };
            ray_street_hit(ray, seg_tree, bldg_tree, ray_hits[i], eps);
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
                ray_hits[pc_endidx].seg->parent_path == ray_hits[pc_startidx].seg->parent_path) {
                pc_endidx++;
            }

            if (pc_startidx < num_rays)
            {
                path_segpiece piece;
                piece.parent_seg = &seg;
                piece.hit_path = ray_hits[pc_startidx].seg->parent_path;

                if (pc_startidx == 0) {
                    piece.start = seg.start.vert;
                    piece.startid = seg.start.id;
                } else {
                    piece.start = segment_idx_mid(ray_origins, pc_startidx - 1, pc_startidx);
                    piece.startid = -1;
                }
                if (pc_endidx == num_rays) {
                    piece.end = seg.end.vert;
                    piece.endid = seg.end.id;
                } else {
                    piece.end = segment_idx_mid(ray_origins, pc_endidx - 1, pc_endidx);
                    piece.endid = -1;
                }

                street_outline_pieces[piece.hit_path].push_back(piece);
            }

            rayidx = pc_endidx;
        }

        rayhit_alloc.deallocate(ray_hits, num_rays);
    };
    

    for (auto& path_segs : footpath_segments)
    {
        for (auto& fseg : path_segs)
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
    }

    // generate the outlines from the pieces, using the start, end ids and parent_seg, and parent_segs prev_seg and next_seg
    // to join the pieces from one segment to those from another segment. Note that the ids are not sequential
    for (auto outline_itr = street_outline_pieces.begin();
        outline_itr != street_outline_pieces.end(); ++outline_itr)
    {
        auto& street = *outline_itr->first;
        auto& street_outline_segs = outline_itr->second;

        //if (street.net_path.type == WAY_TYPE_STREET) {
        //    auto itr = std::find_if(street.net_path.nodes.begin(), street.net_path.nodes.end(),
        //        [](auto& n) { return n.id == 8939625159; });
        //
        //    if (itr != street.net_path.nodes.end()) {
        //        logMESSAGE("Found street node with id 8939625159");
        //    }
        //
        //    //8939625159
        //    //344477816
        //    auto itr2 = std::find_if(street.segments.begin(), street.segments.end(),
        //        [](auto& seg) { return seg.startid == 8939625159 && seg.endid == 344477816; });
        //    if (itr2 != street.segments.end()) {
        //        logMESSAGE("Found street segment with ids 8939625159 and 344477816");
        //    }
        //
        //}


        // connect the segments in order, by checking the end of one segment to the start of another
        std::vector<std::vector<std::pair<osmium::object_id_type, glm::dvec2>>> outlines;
        std::vector<bool> seg_used(street_outline_segs.size(), false);

        for (size_t i = 0; i < street_outline_segs.size(); ++i)
        {
            if (seg_used[i]) {
                continue;
            }
            std::vector<std::pair<osmium::object_id_type, glm::dvec2>> outline;

            outline.push_back({ street_outline_segs[i].startid, street_outline_segs[i].start });
            outline.push_back({ street_outline_segs[i].endid, street_outline_segs[i].end });

            seg_used[i] = true;

            bool extended = true;
            while (extended)
            {
                extended = false;
                // try to extend at the back
                for (size_t j = 0; j < street_outline_segs.size(); ++j)
                {
                    if (seg_used[j]) {
                        continue;
                    }
                    if (street_outline_segs[j].startid == outline.back().first) {
                        outline.push_back({ street_outline_segs[j].endid, street_outline_segs[j].end });
                        seg_used[j] = true;
                        extended = true;
                        break;
                    }
                    else if (street_outline_segs[j].endid == outline.back().first) {
                        outline.push_back({ street_outline_segs[j].startid, street_outline_segs[j].start });
                        seg_used[j] = true;
                        extended = true;
                        break;
                    }
                }

                if (extended) {
                    continue;
                }

                // try to extend at the front
                for (size_t j = 0; j < street_outline_segs.size(); ++j)
                {
                    if (seg_used[j]) {
                        continue;
                    }
                    if (street_outline_segs[j].endid == outline.front().first) {
                        outline.insert(outline.begin(), { street_outline_segs[j].startid, street_outline_segs[j].start });
                        seg_used[j] = true;
                        extended = true;
                        break;
                    }
                    else if (street_outline_segs[j].startid == outline.front().first) {
                        outline.insert(outline.begin(), { street_outline_segs[j].endid, street_outline_segs[i].end });
                        seg_used[j] = true;
                        extended = true;
                        break;
                    }
                }
            }
            outlines.push_back(std::move(outline));
        }

        if (outlines.size() > 2) {
            logMESSAGE("stopping here");
        }

        // keep the two longest outlines
        std::sort(outlines.begin(), outlines.end(),
            [](const auto& a, const auto& b) {
                return a.size() > b.size();
            });
        if (outlines.size() > 2) {
            outlines.resize(2);
        }

        // join the outlines into a single oriented polygon in an order that doesn't intersect itself
        // and triangulate it

        std::vector<std::pair<osmium::object_id_type, glm::dvec2>> joined_outline;

        if (outlines.size() == 2) {
            //auto segment_intersects_outline = [](std::vector<std::pair<osmium::object_id_type, glm::dvec2>>& outline, const segment& seg, double eps) -> bool
            //    {
            //        for (size_t i = 0; i < outline.size() - 1; ++i)
            //        {
            //            seg_inter_result result;
            //            segment outline_seg = { outline[i].second, outline[i + 1].second };
            //            if (segments_intersect(seg, outline_seg, result, eps)) {
            //                return true;
            //            }
            //        }
            //        return false;
            //    };
            //
            //bool found_join = false;
            //if (!segment_intersects_outline(outlines[0], { outlines[0].back().second, outlines[1].front().second }, eps)) {
            //    found_join = true;
            //}
            //if (!found_join && !segment_intersects_outline(outlines[0], { outlines[0].back().second, outlines[1].back().second }, eps)) {
            //    found_join = true;
            //    std::reverse(outlines[1].begin(), outlines[1].end());
            //}
            //
            //joined_outline = std::move(outlines[0]);
            //if (found_join) {
            //    joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
            //}
            //else {
            //    logWARNING("Could not find a way to join the two outlines without intersection. Just appending.");
            //    // just append, will triangulate anyway
            //    joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
            //}

            double dist1 = vec_sqlength(outlines[0].back().second - outlines[1].front().second);
            double dist2 = vec_sqlength(outlines[0].back().second - outlines[1].back().second);

            if (dist1 < dist2) {
                // join the outlines by connecting the last point of the first outline to the first point of the second outline
                joined_outline = std::move(outlines[0]);
                //joined_outline.back() = { outlines[1].front().first, outlines[1].front().second };
                joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
            }
            else {
                // join the outlines by connecting the last point of the first outline to the last point of the second outline
                joined_outline = std::move(outlines[0]);
                std::reverse(outlines[1].begin(), outlines[1].end());
                joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());

                //joined_outline.back() = { outlines[1].back().first, outlines[1].back().second };
                //joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end() - 1);
            }
        }
        else if (outlines.size() == 1) {
            joined_outline = std::move(outlines[0]);
        }
        else {
            continue; // skip if no outlines
        }

        street_outlines[&street] = std::move(joined_outline);
    }

    return street_outlines;
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
    // todo: use each way's width when triangulating the polyline
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

    auto street_outlines_map = gen_street_outlines(footpaths, streets, bldg_tree, eps);



    draw_datad footpath_dd{ 
        .name = "footpaths",
        .color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f)
    };
    draw_datad street_dd{ 
        .name = "streets",
        .color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f)
    };

    // for each footpath, generate drawdata
    for (const auto& footpath : footpaths)
    {
        gen_path_drawdata(footpath_dd, footpath, eps);
    }

    for (auto outline_itr = street_outlines_map.begin(); 
         outline_itr != street_outlines_map.end(); ++outline_itr)
    {
    //    auto& street = *outline_itr->first;
    //    auto& street_outline_segs = outline_itr->second;
    //
    //    if (street.net_path.type == WAY_TYPE_STREET) {
    //        auto itr = std::find_if(street.net_path.nodes.begin(), street.net_path.nodes.end(),
    //            [](auto& n) { return n.id == 8939625159; });
    //
    //        if (itr != street.net_path.nodes.end()) {
    //            logMESSAGE("Found street node with id 8939625159");
    //        }
    //
    //        //8939625159
    //        //344477816
    //        auto itr2 = std::find_if(street.segments.begin(), street.segments.end(),
    //            [](auto& seg) { return seg.startid == 8939625159 && seg.endid == 344477816; });
    //        if (itr2 != street.segments.end()) {
    //            logMESSAGE("Found street segment with ids 8939625159 and 344477816");
    //        }
    //
    //    }
    //    
    //
    //    // connect the segments in order, by checking the end of one segment to the start of another
    //    std::vector<std::vector<std::pair<osmium::object_id_type, glm::dvec2>>> outlines;
    //    std::vector<bool> seg_used(street_outline_segs.size(), false);
    //
    //    for (size_t i = 0; i < street_outline_segs.size(); ++i)
    //    {
    //        if (seg_used[i]) {
    //            continue;
    //        }
    //        std::vector<std::pair<osmium::object_id_type, glm::dvec2>> outline;
    //
    //        outline.push_back({ street_outline_segs[i]->startid, street_outline_segs[i]->start });
    //        outline.push_back({ street_outline_segs[i]->endid, street_outline_segs[i]->end });
    //        
    //        seg_used[i] = true;
    //        
    //        bool extended = true;
    //        while (extended)
    //        {
    //            extended = false;
    //            // try to extend at the back
    //            for (size_t j = 0; j < street_outline_segs.size(); ++j)
    //            {
    //                if (seg_used[j]) {
    //                    continue;
    //                }
    //                if (street_outline_segs[j]->startid == outline.back().first) {
    //                    outline.push_back({ street_outline_segs[j]->endid, street_outline_segs[j]->end });
    //                    seg_used[j] = true;
    //                    extended = true;
    //                    break;
    //                }
    //                else if (street_outline_segs[j]->endid == outline.back().first) {
    //                    outline.push_back({ street_outline_segs[j]->startid, street_outline_segs[j]->start });
    //                    seg_used[j] = true;
    //                    extended = true;
    //                    break;
    //                }
    //            }
    //
    //            if (extended) {
    //                continue;
    //            }
    //
    //            // try to extend at the front
    //            for (size_t j = 0; j < street_outline_segs.size(); ++j)
    //            {
    //                if (seg_used[j]) {
    //                    continue;
    //                }
    //                if (street_outline_segs[j]->endid == outline.front().first) {
    //                    outline.insert(outline.begin(), { street_outline_segs[j]->startid, street_outline_segs[j]->start });
    //                    seg_used[j] = true;
    //                    extended = true;
    //                    break;
    //                }
    //                else if (street_outline_segs[j]->startid == outline.front().first) {
    //                    outline.insert(outline.begin(), { street_outline_segs[j]->endid, street_outline_segs[i]->end });
    //                    seg_used[j] = true;
    //                    extended = true;
    //                    break;
    //                }
    //            }
    //        }
    //        outlines.push_back(std::move(outline));
    //    }
    //
    //    // keep the two longest outlines
    //    std::sort(outlines.begin(), outlines.end(),
    //        [](const auto& a, const auto& b) {
    //            return a.size() > b.size();
    //        });
    //    if (outlines.size() > 2) {
    //        outlines.resize(2);
    //    }
    //
    //    // join the outlines into a single oriented polygon in an order that doesn't intersect itself
    //    // and triangulate it
    //    
    //    std::vector<std::pair<osmium::object_id_type, glm::dvec2>> joined_outline;
    //
    //    if (outlines.size() == 2) {
    //        //auto segment_intersects_outline = [](std::vector<std::pair<osmium::object_id_type, glm::dvec2>>& outline, /const /segment& seg, double eps) -> bool
    //        //    {
    //        //        for (size_t i = 0; i < outline.size() - 1; ++i)
    //        //        {
    //        //            seg_inter_result result;
    //        //            segment outline_seg = { outline[i].second, outline[i + 1].second };
    //        //            if (segments_intersect(seg, outline_seg, result, eps)) {
    //        //                return true;
    //        //            }
    //        //        }
    //        //        return false;
    //        //    };
    //        //
    //        //bool found_join = false;
    //        //if (!segment_intersects_outline(outlines[0], { outlines[0].back().second, outlines[1].front().second }, eps)) {
    //        //    found_join = true;
    //        //}
    //        //if (!found_join && !segment_intersects_outline(outlines[0], { outlines[0].back().second, outlines[1].back//().second }, eps)) {
    //        //    found_join = true;
    //        //    std::reverse(outlines[1].begin(), outlines[1].end());
    //        //}
    //        //
    //        //joined_outline = std::move(outlines[0]);
    //        //if (found_join) {
    //        //    joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
    //        //}
    //        //else {
    //        //    logWARNING("Could not find a way to join the two outlines without intersection. Just appending.");
    //        //    // just append, will triangulate anyway
    //        //    joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
    //        //}
    //
    //        double dist1 = vec_sqlength(outlines[0].back().second - outlines[1].front().second);
    //        double dist2 = vec_sqlength(outlines[0].back().second - outlines[1].back().second);
    //
    //        if (dist1 < dist2) {
    //            // join the outlines by connecting the last point of the first outline to the first point of the second /outline
    //            joined_outline = std::move(outlines[0]);
    //            //joined_outline.back() = { outlines[1].front().first, outlines[1].front().second };
    //            joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
    //        }
    //        else {
    //            // join the outlines by connecting the last point of the first outline to the last point of the second outline
    //            joined_outline = std::move(outlines[0]);
    //            std::reverse(outlines[1].begin(), outlines[1].end());
    //            joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end());
    //
    //            //joined_outline.back() = { outlines[1].back().first, outlines[1].back().second };
    //            //joined_outline.insert(joined_outline.end(), outlines[1].begin(), outlines[1].end() - 1);
    //        }
    //    }
    //    else if (outlines.size() == 1) {
    //        joined_outline = std::move(outlines[0]);
    //    }
    //    else {
    //        continue; // skip if no outlines
    //    }
        //for (size_t i = 0; i < outlines.size(); ++i)
        //{
        //    auto& outline = outlines[i];
        //
        //    // check if the outline is oriented clockwise or counter-clockwise
        //    std::vector<glm::dvec2> outline_verts;
        //    outline_verts.reserve(outline.size());
        //    for (auto & point : outline) {
        //        outline_verts.push_back(point.second);
        //    }
        //
        //    bool is_anticlockwise = polygon_orient(outline_verts) == ORIENT_CCW;
        //    if (is_anticlockwise) {
        //        // reverse the outline to make it clockwise
        //        std::reverse(outline.begin(), outline.end());
        //    }
        //
        //    // add the outline to the joined outline
        //    if (joined_outline.empty()) {
        //        joined_outline = std::move(outline);
        //    }
        //    else {
        //        //// check if the last point of the joined outline is the same as the first point of the outline
        //        //if (joined_outline.back().first == outline.front().first) {
        //        //    joined_outline.back().second = outline.front().second; // update the last point
        //        //    joined_outline.insert(joined_outline.end(), outline.begin() + 1, outline.end());
        //        //}
        //        //else if (joined_outline.front().first == outline.back().first) {
        //        //    joined_outline.front().second = outline.back().second; // update the first point
        //        //    joined_outline.insert(joined_outline.begin(), outline.begin(), outline.end() - 1);
        //        //}
        //        //else {
        //        //    // no connection, just append
        //        //    
        //        //}
        //        joined_outline.insert(joined_outline.end(), outline.begin(), outline.end());
        //    }
        //}

        auto joined_outline = outline_itr->second;

        if (joined_outline.size() < 3) {
            continue; // skip outlines with less than 3 points
        }
        // triangulate the joined outline
        std::vector<glm::dvec2> joined_outline_verts;
        joined_outline_verts.reserve(joined_outline.size());
        for (const auto& point : joined_outline) {
            joined_outline_verts.push_back(point.second);
        }

        auto joined_orient = polygon_orient(joined_outline_verts);

        auto tri_indices = polygon_triangulate(joined_outline_verts, joined_orient == ORIENT_CCW);
        if (tri_indices.empty()) {
            continue; // skip outlines with no triangles
        }
        uint32_t vert_startidx = uint32_t(street_dd.num_verts());
        for (const auto& point : joined_outline) {
            street_dd.add_vertex(point.second.x, point.second.y, 0.0);
        }
        for (size_t i = 0; i < tri_indices.size(); i += 3) {
            street_dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
        }
    }

    drawdata.push_back(std::move(footpath_dd));
    drawdata.push_back(std::move(street_dd));

    return true;
}
