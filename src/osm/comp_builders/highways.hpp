
#ifndef OSM_MESH_STREETS_HPP
#define OSM_MESH_STREETS_HPP

#include "types.hpp"
#include "buildings.hpp"

// OSM concept of a "highway".
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

struct highway_comp
{
    int mesh_obj_idx;
    highway_type type;
    double width;

    struct way_net_traits
    {
        using way_enum_type = highway_type;

        static way_enum_type way_type(const highway_comp* comp) {
            return comp->type;
        }
    };
};

struct highway_comp_builder
{
    using comp_type = highway_comp;
    constexpr const char* comp_type_name() const { return "highway"; }

    template <class ...TComps>
    bool add_comp(int mesh_obj_idx, const osmium::OSMObject* obj,
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info)
    {
        if (obj->type() == osmium::item_type::way)
        {
            auto* way = static_cast<const osmium::Way*>(obj);
            assert(!way->is_closed()); // closed ways should be areas

            int type = get_highway_type(*way);
            if (type != -1)
            {
                auto htype = (highway_type)type;
                auto& comps_vec = comp_db->comps_vec<highway_comp>();

                comps_vec.push_back({
                    .mesh_obj_idx = mesh_obj_idx,
                    .type = htype,
                    .width = get_highway_width(*way, htype)
                });
                comp_info.push_back({
                    .type = COMP_TYPE_STREET,
                    .comp_idx = int(comps_vec.size() - 1)
                });

                m_num_hiway_nodes += way->nodes().size();
                return true;
            }
        }
        // todo: handle areas
        return false;
    }

    template <class ...TComps>
    bool build_all(const osm_mesh_object_db* obj_db,
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<osm_tri_datad>& out_drawdata)
    {
        auto& streets = comp_db->comps_vec<highway_comp>();
        auto& bldgs = comp_db->comps_vec<building_comp>();
        return do_build_all(obj_db, streets, bldgs, out_drawdata);
    }

private:
    bool do_build_all(const osm_mesh_object_db* obj_db, const std::vector<highway_comp>& highways,
        const std::vector<building_comp>& buildings, std::vector<osm_tri_datad>& out_drawdata);

    int get_highway_type(const osmium::Way& way);
    double get_highway_width(const osmium::Way& way, highway_type type);

    int m_num_hiway_nodes = 0;
};

#endif