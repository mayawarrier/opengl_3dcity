#include "mesh.hpp"

std::vector<draw_datad> mesh_builder::get_draw_data()
{
    std::vector<draw_datad> ret;
    
    add_building_drawdata(ret);
    add_street_drawdata(ret);

    bbox3d bbox = center_drawdata_batch(ret);

    //// add ground plane
    //draw_datad ground_dd;
    //ground_dd.color = { 0.2f, 0.2f, 0.2f, 1.0f };
    //ground_dd.name = "ground plane";
    //ground_dd.verts = {
    //    bbox.min.x, bbox.min.y, bbox.min.z - 0.1,
    //    bbox.max.x, bbox.min.y, bbox.min.z - 0.1,
    //    bbox.max.x, bbox.max.y, bbox.min.z - 0.1,
    //    bbox.min.x, bbox.max.y, bbox.min.z - 0.1
    //};
    //ground_dd.tri_indices = { 0, 2, 1, 0, 3, 2 };
    //
    //ret.push_back(std::move(ground_dd));

    return ret;
}

