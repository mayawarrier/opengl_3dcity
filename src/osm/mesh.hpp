#ifndef OSM_MESH_HPP
#define OSM_MESH_HPP

#include <string>
#include <vector>

#include <osmium/osm/types.hpp>
#include <osmium/fwd.hpp>

#include "geom.hpp"


enum highway_type
{
    HIGHWAY_TYPE_UNKNOWN,
    HIGHWAY_TYPE_STREET,
    HIGHWAY_TYPE_FOOTWAY
};

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

    struct highway_info
    {
        way_info way;
        highway_type type;
        int lanes; // number of lanes (-1 if not present)
        double width; // in meters, estimate if not present
    };

    bool add_building(const building_info& info);

    // OSM concept of "highway".
    bool add_highway(const highway_info& info);

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
        highway_type type;
        std::vector<way_node> nodes;
        double width;
    };

private:
    bool get_building_part(const building_info& info, building_part& part);

    bool add_building_drawdata(std::vector<draw_datad>& drawdata);
    bool add_street_drawdata(std::vector<draw_datad>& drawdata);

    std::vector<building> m_buildings;
    std::vector<building_part> m_building_parts;
    std::vector<thick_way> m_highways;
};

#endif
