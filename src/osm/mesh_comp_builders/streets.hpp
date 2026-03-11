
#ifndef OSM_MESH_STREETS_HPP
#define OSM_MESH_STREETS_HPP

#include "types.hpp"

// https://wiki.openstreetmap.org/wiki/Highways
enum highway_type
{
    HIGHWAY_TYPE_UNKNOWN,
    HIGHWAY_TYPE_MOTORWAY,
    HIGHWAY_TYPE_TRUNK,
    HIGHWAY_TYPE_PRIMARY,
    HIGHWAY_TYPE_SECONDARY,
    HIGHWAY_TYPE_TERTIARY,
    HIGHWAY_TYPE_UNCLASSIFIED,
    HIGHWAY_TYPE_RESIDENTIAL,
    HIGHWAY_TYPE_MOTORWAY_LINK,
    HIGHWAY_TYPE_TRUNK_LINK,
    HIGHWAY_TYPE_PRIMARY_LINK,
    HIGHWAY_TYPE_SECONDARY_LINK,
    HIGHWAY_TYPE_TERTIARY_LINK,
    HIGHWAY_TYPE_LIVING_STREET,
    HIGHWAY_TYPE_SERVICE,
    HIGHWAY_TYPE_PEDESTRIAN,
    HIGHWAY_TYPE_ROAD,
    HIGHWAY_TYPE_FOOTWAY, // generic footway
    HIGHWAY_TYPE_FOOTWAY_SIDEWALK, // footway=sidewalk tag
    HIGHWAY_TYPE_FOOTWAY_CROSSING, // footway=crossing tag
};

struct street_comp
{
    int mesh_obj_idx;
    highway_type type;
    double width;

    struct way_net_traits
    {
        using way_enum_type = highway_type;

        static way_enum_type way_type(const street_comp* comp) {
            return comp->type;
        }
    };
};

struct street_comp_builder
{
    using comp_type = street_comp;
    constexpr const char* comp_type_name() const { return "street"; }

    template <class ...TComps>
    bool add_comp(int mesh_obj_idx, const osmium::OSMObject* obj, 
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info);

    template <class ...TComps>
    bool build_all(const osm_mesh_object_db* obj_db, 
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<draw_datad>& out_drawdata);
};

#include "streets_impl.hpp"

#endif