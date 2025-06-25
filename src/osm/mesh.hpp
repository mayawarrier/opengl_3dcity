
#ifndef AABB_TREE_HPP
#define AABB_TREE_HPP

#include <string>
#include <vector>

#include <osmium/osm/types.hpp>
#include <osmium/fwd.hpp>

#include "geom.hpp"

template <typename TVert>
struct draw_data
{
    std::string name;
    std::vector<TVert> verts;
    std::vector<uint32_t> tri_indices;
    std::vector<uint32_t> line_indices;
};

using draw_dataf = draw_data<float>;
using draw_datad = draw_data<double>;


// Processes OSM data into a set of meshes/lines 
// that can be rendered by opengl.
class mesh_builder
{
public:
    struct way_info
    {
        osmium::object_id_type id;
        const char* name;
        const osmium::NodeRefList& nodes;
    };

    struct building_info
    {
        way_info way;
        bool is_part;
        double ht_btm, ht_top; // in meters
    };

    struct street_info
    {
        way_info way;
        double width; // in meters
    };

    bool add_building(const building_info& info);

    bool add_street(const street_info& info);

    std::vector<draw_datad> get_draw_data();

public:
    struct building_part
    {
        osmium::object_id_type id;
        orient orient;
        bbox2d bbox;
        double ht_btm, ht_top;
        std::vector<osmpoint> verts;
    };

    struct building
    {
        building_part info;
        std::string name;
        std::vector<building_part*> parts;
    };

    struct way_node
    {
        osmium::object_id_type id;
        osmpoint vert;
    };

    struct street_way
    {
        osmium::object_id_type id;
        std::string name;
        double width;
        std::vector<way_node> nodes;
    };

private:
    bool get_building_part(const building_info& info, building_part& part);

    bool add_building_drawdata(std::vector<draw_datad>& drawdata);
    bool add_street_drawdata(std::vector<draw_datad>& drawdata);

    std::vector<building> m_buildings;
    std::vector<building_part> m_building_parts;
    std::vector<street_way> m_streetways;
};

#endif
