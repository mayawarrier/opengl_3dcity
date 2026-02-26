#ifndef OSM_MESH_HPP
#define OSM_MESH_HPP

#include <vector>

#include <osmium/osm/types.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/area.hpp>

#include "containers/aabb_tree.hpp"
#include "common.hpp"


struct mesh_object;

struct mesh_component
{
    enum type_t
    {
        COMP_TYPE_HIGHWAY,
        COMP_TYPE_BUILDING,
        COMP_TYPE_BUILDING_PART,
        COMP_TYPE_POI
    } type;
    int subtype; // e.g. highway type
    int parent_obj_idx;
};

struct mesh_object
{
    using comp_vec_t = types::small_vector<mesh_component, 1>;

    enum type_t
    {
        OBJ_TYPE_NODE,
        OBJ_TYPE_WAY,
        OBJ_TYPE_AREA
    } type;

    int osm_obj_idx;
    bbox2d bbox;
    comp_vec_t comps;
    std::string name;

    struct aabb_traits {
        static bbox2d bbox(mesh_object* comp) {
            return comp->bbox;
        }
    };
};

// Converts OSM data into triangles for rendering.
class mesh_builder
{
public:
    // Add node. All nodes must be added before all ways/areas.
    bool add_node(const osmium::Node& node);

    // Add linear way. Closed ways should use add_area().
    bool add_way(const osmium::Way& way);

    // Add area.
    bool add_area(const osmium::Area& area);

    // Build meshes.
    void build();

    const std::vector<draw_datad>& draw_data() const {
        return m_drawdata; 
    }

private:
    void lock_nodes();
    osm_node nr_to_osm_node(const osmium::NodeRef& nr);
    osm_node nr_to_osm_node(const osmium::NodeRef& nr, bbox2d& bbox);
    
    bool add_street_comp(const osmium::OSMObject* obj, mesh_object::comp_vec_t& comps);
    bool add_bldg_comp(const osmium::OSMObject* obj, mesh_object::comp_vec_t& comps);

    bool build_streets();
    bool build_buildings();

    std::vector<osm_node> m_nodes;
    std::vector<osm_way> m_ways;
    std::vector<osm_area> m_areas;

    std::vector<mesh_object> m_objects;
    aabb_tree2d<mesh_object*> m_obj_tree;

    glm::dvec2 m_center{ 0.0, 0.0 };
    std::size_t m_num_nodes = 0;
    bool m_nodes_locked = false;

    std::vector<draw_datad> m_drawdata;
};

#endif
