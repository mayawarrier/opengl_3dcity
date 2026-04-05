
#include "../geom_impl.hpp"
#include "../mesh_builder.hpp"

#include "water.hpp"

static void gen_path_drawdata(osm_tri_datad& dd, const way_network<mesh_entity>::path& path,
    const mesh_entity_db* entity_db, const std::vector<water_comp>& waters, double eps)
{
    std::vector<glm::dvec2> verts(path.nodes.size());
    for (size_t i = 0; i < path.nodes.size(); ++i) {
        verts[i] = path.nodes[i].vert();
    }
    // todo: use each way's width when triangulating the polyline
    int comp_idx = entity_db->ent_comp_idx(*path.nodes[1].in_way, COMP_TYPE_WATER);
    const auto& water_comp = waters[comp_idx];
    polyline_triangulate(verts, water_comp.width, dd, TRI_TYPE_WATER, eps);
}

static bool gen_area_drawdata(osm_tri_datad& dd, const osm_area& area)
{
    for (const auto& poly : area.polys) {

        auto tri_indices = polygon_triangulate(poly);
        auto poly_nodes = osm_area::poly_nodes(poly);

        dd_add_polygon(dd, poly_nodes, tri_indices, 0.0, TRI_TYPE_WATER);
    }
    return true;
}

bool water_comp_builder::do_build_all(const mesh_entity_db* entity_db,
    const std::vector<water_comp>& waters, std::vector<osm_tri_datad>& out_tridata)
{
    static constexpr double eps = 1e-9;

    auto tbegin = clk::now();

    auto timed_section = [&](const char* name, auto&& func) {
        log_func(name, std::forward<decltype(func)>(func), "Water mesher");
    };

    std::vector<way_network<mesh_entity>::path> all_paths;
    timed_section("Extracting paths", [&]()
    {
        all_paths = entity_db->way_net.get_all_paths_to_intersections(WAY_COMP_TYPE_WATERWAY, [&](const mesh_entity* ent) {
            int comp_idx = entity_db->ent_comp_idx(*ent, COMP_TYPE_WATER);
            return waters[comp_idx].type;
        });
    });

    osm_tri_datad dd;
    timed_section("Triangulating paths", [&]() {
        for (const auto& path : all_paths) {
            gen_path_drawdata(dd, path, entity_db, waters, eps);
        }
    });

    timed_section("Triangulating areas", [&]()
    {
        for (const auto& water_comp : waters) {
            if (water_comp.type == WATER_TYPE_AREA) {
                const auto& ent = entity_db->get<mesh_entity>(water_comp.entity_idx);
                const auto& area = entity_db->get<osm_area>(ent.obj_idx);
                gen_area_drawdata(dd, area);
            }
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
