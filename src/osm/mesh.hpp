
#ifndef MESH_HPP
#define MESH_HPP

#include <string>
#include <vector>

#include <osmium/osm/types.hpp>
#include <osmium/fwd.hpp>

#include "geom.hpp"


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
    // for now
    using footpath_info = street_info;

    bool add_building(const building_info& info);

    bool add_street(const street_info& info);
    bool add_footpath(const footpath_info& info);

    std::vector<draw_datad> get_draw_data();

public:
    struct building_part
    {
        osmium::object_id_type id;
        orient_t orient;
        bbox2d bbox;
        double ht_btm, ht_top;
        std::vector<glm::dvec2> verts;
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
        glm::dvec2 vert;
    };

    struct thick_way
    {
        osmium::object_id_type id;
        std::string name;
        double width;
        std::vector<way_node> nodes;
    };

private:
    bool get_building_part(const building_info& info, building_part& part);
    thick_way get_thick_way(const way_info& info, double width);

    bool add_building_drawdata(std::vector<draw_datad>& drawdata);
    bool add_street_drawdata(std::vector<draw_datad>& drawdata, 
        const std::vector<thick_way>& ways, const glm::vec4& color);

    std::vector<building> m_buildings;
    std::vector<building_part> m_building_parts;
    std::vector<thick_way> m_streetways;
    std::vector<thick_way> m_footways;
};

#endif
