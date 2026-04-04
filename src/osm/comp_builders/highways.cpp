
#include "../geom/geom.hpp"
#include "../mesh_builder.hpp"

#include "highways.hpp"


//using way_net = way_network<highway_comp>;
//
//static inline int path_num_segs(const way_net::path* path) {
//    return int(path->nodes.size()) - 1;
//}
//
//static inline segment path_seg(const way_net::path* path, int idx) {
//    return { path->nodes[idx].vert(), path->nodes[idx + 1].vert() };
//}
//
//static inline double path_seg_width(const way_net::path* path, int idx) {
//    // +1 because in_way is null for the first node
//    return path->nodes[idx + 1].in_way->width;
//}
//
//static inline glm::dvec2 path_point(const way_net::path* path, int seg_idx, double seg_param) {
//    return seg_at_param(path_seg(path, seg_idx), seg_param);
//}
//
//static inline bool path_has_node(const way_net::path* path, osmium::object_id_type id) {
//    auto& c = path->nodes;
//    return std::find_if(c.begin(), c.end(), [&](auto& n) { return n.id() == id; }) != c.end();
//}

static void gen_path_drawdata(osm_tri_datad& dd, const way_network<mesh_entity>::path& path, 
    const mesh_entity_db* entity_db, const std::vector<highway_comp>& highways, double eps)
{
    std::vector<glm::dvec2> verts(path.nodes.size());
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        verts[i] = path.nodes[i].vert();
    }
    // todo: use each way's width when triangulating the polyline
    int comp_idx = entity_db->ent_comp_idx(*path.nodes[1].in_way, COMP_TYPE_HIGHWAY);
    const auto& hw_comp = highways[comp_idx];
    polyline_triangulate(verts, hw_comp.width, dd, TRI_TYPE_HIGHWAY, eps);
}

//static bool gen_outline_drawdata(draw_datad& dd, const std::vector<outline_node>& outline)
//{
//    std::vector<glm::dvec2> outline_verts(outline.size());
//    for (size_t i = 0; i < outline.size(); ++i) {
//        outline_verts[i] = outline[i].vert;
//    }
//    if (path_orient(outline_verts) == ORIENT_CW) {
//        std::reverse(outline_verts.begin(), outline_verts.end());
//    }
//    
//    auto vert_span = std::span<const glm::dvec2>(outline_verts);
//    auto tri_indices = polygon_triangulate(std::span(&vert_span, 1));
//    if (tri_indices.empty()) {
//        logDEBUG(LOG_MESSAGE, "Skipping outline since it has no triangles");
//        return false;
//    }
//    assert(check_triangles_oriented(outline, tri_indices));
//
//    uint32_t vert_startidx = dd.num_verts();
//    for (const auto& point : outline_verts) {
//        dd.add_vertex(point.x, point.y, 0.0);
//    }
//    for (size_t i = 0; i < tri_indices.size(); i += 3) {
//        dd.add_triangle_w_offset(tri_indices[i], tri_indices[i + 1], tri_indices[i + 2], vert_startidx);
//    }
//
//    return true;
//}

bool highway_comp_builder::do_build_all(const mesh_entity_db* entity_db, 
    const std::vector<highway_comp>& highways, const std::vector<building_comp>& buildings, 
    std::vector<osm_tri_datad>& out_tridata)
{
    static constexpr double eps = 1e-9;
    
    auto tbegin = clk::now();

    logMESSAGE("%zu ways, %zu nodes", highways.size(), m_num_hiway_nodes);

    auto timed_section = [&](const char* name, auto&& func) {
        log_func(name, std::forward<decltype(func)>(func), "Highway mesher");
    };
    
    std::vector<way_network<mesh_entity>::path> all_paths;
    timed_section("Extracting paths", [&]() 
    {
        all_paths = entity_db->way_net.get_all_paths_to_intersections(WAY_COMP_TYPE_HIGHWAY, [&](const mesh_entity* ent) {
            int comp_idx = entity_db->ent_comp_idx(*ent, COMP_TYPE_HIGHWAY);
            return highways[comp_idx].type;
        });
    });

    osm_tri_datad dd;
    timed_section("Triangulating paths", [&]() {
        for (const auto& street : all_paths) {
            gen_path_drawdata(dd, street, entity_db, highways, eps);
        }
    });

    uint32_t num_tris = dd.num_tris();
    uint32_t num_verts = dd.num_verts();
    out_tridata.push_back(std::move(dd));

    auto tend = clk::now();

    logMESSAGE("Generated %u tris and %u vertices in %s", 
        num_tris, num_verts, clock_dur_str(tend - tbegin).c_str());

    return true;
}
