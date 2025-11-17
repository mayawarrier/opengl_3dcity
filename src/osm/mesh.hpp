#ifndef OSM_MESH_HPP
#define OSM_MESH_HPP

#include <string>
#include <vector>
#include <variant>

#include <osmium/osm/types.hpp>
#include <osmium/memory/item_iterator.hpp>
#include <osmium/fwd.hpp>
#include <boost/unordered/unordered_flat_map.hpp>

#include "containers/fwd.hpp"
#include "common.hpp"


// Processes OSM data into a set of meshes/triangles
// that can be rendered by opengl.
class mesh_builder
{
public:
    struct building_info
    {
        object_type obj_type;
        bool is_part;
        union
        {
            const osmium::Way* way;
            const osmium::Area* area;
        };
    };

    struct highway_info
    {
        const osmium::Way* way;
        way_type type;
        int lanes; // number of lanes (-1 if not present)
        double width; // in meters, estimate if not present
    };

    bool add_building(const building_info& info);

    // OSM concept of "highway".
    bool add_highway(const highway_info& info);

    std::vector<draw_datad> get_draw_data();

public:
    struct area
    {
        std::vector<std::span<const glm::dvec2>> rings;
    };

    struct building_part
    {
        osmium::object_id_type id;
        std::vector<glm::dvec2> verts;
        bbox2d bbox;
        double ht_btm, ht_top;
        // False if default height used.
        bool has_ht_btm, has_ht_top;
    };

    struct building
    {
        building_part base;
        std::string name;
        std::vector<building_part*> parts;

        struct aabb_traits
        {
            static const bbox2d& bbox(building* building) {
                return building->base.bbox;
            }
        };
    };

    struct highway
    {
        osmium::object_id_type id;
        std::string name;
        way_type type;
        std::vector<osm_node> nodes;
        double width;

        struct way_net_traits
        {
            static way_type way_type(const highway* way) {
                return way->type;
            }
        };
    };

private:
    bool get_building_part(const building_info& info, building_part& part);

    bool gen_building_drawdata(std::vector<draw_datad>& drawdata, aabb_tree2d<building*>* out_bldg_tree);
    bool gen_street_drawdata(std::vector<draw_datad>& drawdata, const aabb_tree2d<building*>* bldg_tree);

    std::vector<building> m_buildings;
    std::vector<building_part> m_building_parts;
    types::unord_flat_map<osmium::object_id_type, area> m_bldg_areas;
    std::vector<highway> m_highways;
    std::size_t m_num_highway_nodes = 0;
};

#endif
