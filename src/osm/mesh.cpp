#include "mesh.hpp"

static bbox3d center_drawdata_batch(std::span<draw_datad> batch)
{
    bbox3d batch_bbox;
    for (draw_datad& dd : batch) {
        for (size_t i = 0; i < dd.verts.size(); i += 3) {
            batch_bbox.extend({
                dd.verts[i + 0],
                dd.verts[i + 1],
                dd.verts[i + 2]
            });
        }
    }
    glm::dvec3 batch_center = batch_bbox.center();
    for (draw_datad& dd : batch) {
        for (size_t i = 0; i < dd.verts.size(); i += 3) {
            dd.verts[i + 0] -= batch_center.x;
            dd.verts[i + 1] -= batch_center.y;
            dd.verts[i + 2] -= batch_center.z;
        }
    }

    bbox3d ret;
    ret.min = batch_bbox.min - batch_center;
    ret.max = batch_bbox.max - batch_center;
    return ret;
}


std::vector<draw_datad> mesh_builder::get_draw_data()
{
    aabb_tree<building*> bldg_tree;

    std::vector<draw_datad> ret;
    //gen_building_drawdata(ret, bldg_tree);
    gen_street_drawdata(ret, bldg_tree);

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

