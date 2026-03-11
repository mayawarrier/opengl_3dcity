
#ifndef OSM_MESH_BUILDINGS_HPP
#define OSM_MESH_BUILDINGS_HPP

#include "types.hpp"

struct bldg_comp
{
    int mesh_obj_idx;
    double bldg_ht_btm, bldg_ht_top;
    double roof_ht_top;
};

struct bldg_comp_builder
{
    using comp_type = bldg_comp;
    constexpr const char* comp_type_name() const { return "building"; }

    template <class ...TComps>
    bool add_comp(int mesh_obj_idx, const osmium::OSMObject* obj,
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info);

    template <class ...TComps>
    bool build_all(const osm_mesh_object_db* obj_db,
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<draw_datad>& out_drawdata);
};

#include "buildings_impl.hpp"

#endif