
#ifndef OSM_MESH_WATER_HPP
#define OSM_MESH_WATER_HPP

#include <osmium/osm/area.hpp>

#include "types.hpp"

// https://wiki.openstreetmap.org/wiki/Water
enum water_type
{
    WATER_TYPE_AREA,
    WATER_TYPE_LINE
};

struct water_comp
{
    int mesh_obj_idx;
    water_type type;
};

struct water_comp_builder
{
    using comp_type = water_comp;
    constexpr const char* comp_type_name() const { return "water"; }

    template <class ...TComps>
    bool add_comp(int mesh_obj_idx, const osmium::OSMObject* obj,
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info)
    {
        if (obj->type() == osmium::item_type::area)
        {
            auto* area = static_cast<const osmium::Area*>(obj);

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
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<osm_tri_datad>& out_tridata)
    {
        auto& waters = comp_db->comps_vec<water_comp>();
        return do_build_all(obj_db, waters, out_tridata);
    }

private:
    bool do_build_all(const osm_mesh_object_db* obj_db, 
        const std::vector<water_comp>& waters, std::vector<osm_tri_datad>& out_tridata);

    int get_water_type(const osmium::Area& area) const
    {
        // https://wiki.openstreetmap.org/wiki/Key:natural
        const char* natural = area.tags()["natural"];
        if (natural && std::strcmp(natural, "water") == 0) {
            return WATER_TYPE_AREA;
        }
        // https://wiki.openstreetmap.org/wiki/Key=waterway
        const char* waterway = area.tags()["waterway"];
        if (waterway && (std::strcmp(waterway, "river") == 0 || std::strcmp(waterway, "stream") == 0)) {
            return WATER_TYPE_LINE;
        }
        return -1;
    }
};

#endif