
#ifndef OSM_MESH_BUILDINGS_HPP
#define OSM_MESH_BUILDINGS_HPP

#include "types.hpp"


// todo: add types
// https://wiki.openstreetmap.org/wiki/Buildings
enum building_type
{
    BUILDING_TYPE_YES
};

// todo: add types
// https://wiki.openstreetmap.org/wiki/Key:building:part
enum building_part_type
{
    BUILDING_PART_TYPE_YES
};


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
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info)
    {
        if (obj->type() != osmium::item_type::area) {
            return false;
        }
        auto* area = static_cast<const osmium::Area*>(obj);

        auto& tags = area->tags();
        bool is_building = tags.has_key("building");
        bool is_bldg_part = tags.has_key("building:part");

        if (is_building || is_bldg_part)
        {
            auto type = is_building ?
                osm_mesh_object::comp_info::COMP_TYPE_BUILDING :
                osm_mesh_object::comp_info::COMP_TYPE_BUILDING_PART;

            int subtype = is_building ?
                building_type::BUILDING_TYPE_YES :
                building_part_type::BUILDING_PART_TYPE_YES;

            auto& comps_vec = comp_db->comps_vec<bldg_comp>();
            bldg_comp comp = {
                .mesh_obj_idx = mesh_obj_idx,
            };
            set_bldg_heights(tags, comp);

            comps_vec.push_back(comp);

            comp_info.push_back({
                .type = type,
                .comp_idx = int(comps_vec.size() - 1)
            });
            return true;
        }
        return false;
    }

    template <class ...TComps>
    bool build_all(const osm_mesh_object_db* obj_db,
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<draw_datad>& out_drawdata)
    {
        auto& bldgs = comp_db->comps_vec<bldg_comp>();
        return do_build_all(obj_db, bldgs, out_drawdata);
    }

private:
    bool do_build_all(const osm_mesh_object_db* obj_db, 
        const std::vector<bldg_comp>& bldgs, std::vector<draw_datad>& out_drawdata);

    void set_bldg_heights(const osmium::TagList& tags, bldg_comp& comp);
};


#endif