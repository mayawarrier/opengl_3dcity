
#ifndef OSM_MESH_WATER_HPP
#define OSM_MESH_WATER_HPP

#include <osmium/osm/area.hpp>
#include <osmium/osm/way.hpp>

#include "types.hpp"


// https://wiki.openstreetmap.org/wiki/Water
enum water_type
{
    WATER_TYPE_AREA,
    WATER_TYPE_LINE_RIVER,
    WATER_TYPE_LINE_CANAL,
    WATER_TYPE_LINE_STREAM,
    WATER_TYPE_LINE_DITCH,
    WATER_TYPE_LINE_DRAIN
};

struct water_comp
{
    int entity_idx;
    water_type type;
    double width;
};

struct water_comp_builder
{
    using comp_t = water_comp;
    constexpr const char* comp_type_name() const { return "water"; }

    template <class ...TComps>
    bool add_comp(int entity_idx, const osmium::OSMObject* obj,
        mesh_comp_db<TComps...>* comp_db, mesh_entity::comp_info_vec_t& comp_info)
    {
        bool added = false;
        auto& comps_vec = comp_db->comps_vec<water_comp>();

        if (obj->type() == osmium::item_type::way)
        {
            int type = get_water_line_type(obj->tags());          
            if (type != -1) {
                comps_vec.push_back({
                    .entity_idx = entity_idx,
                    .type = (water_type)type,
                    .width = get_water_line_width(obj->tags(), (water_type)type)
                });
                added = true;
            }
        }
        else if (obj->type() == osmium::item_type::area)
        {
            int type = get_water_area_type(obj->tags());
            if (type != -1) {
                comps_vec.push_back({
                    .entity_idx = entity_idx,
                    .type = (water_type)type,
                    .width = 0.0
                });
                added = true;
            }
        }
        if (added) {
            comp_info.push_back({
                .type = COMP_TYPE_WATER,
                .comp_idx = int(comps_vec.size() - 1)
            });
            return true;
        }
        return false;
    }

    template <class ...TComps>
    bool build_all(const mesh_entity_db* obj_db,
        mesh_comp_db<TComps...>* comp_db, std::vector<osm_tri_datad>& out_tridata)
    {
        auto& waters = comp_db->comps_vec<water_comp>();
        return do_build_all(obj_db, waters, out_tridata);
    }

private:
    bool do_build_all(const mesh_entity_db* obj_db, 
        const std::vector<water_comp>& waters, std::vector<osm_tri_datad>& out_tridata);

    // https://wiki.openstreetmap.org/wiki/Key=waterway
    // https://wiki.openstreetmap.org/wiki/Key=natural
    // https://wiki.openstreetmap.org/wiki/Key=landuse
    int get_water_area_type(const osmium::TagList& tags) const
    {
        if (is_underground(tags)) {
            return -1;
        }
        const char* waterway = tags["waterway"];
        if (waterway) {
            if (std::strcmp(waterway, "riverbank") == 0) return WATER_TYPE_AREA;
            if (std::strcmp(waterway, "dock") == 0)      return WATER_TYPE_AREA;
        }
        const char* landuse = tags["landuse"];
        if (landuse) {
            if (std::strcmp(landuse, "reservoir") == 0) return WATER_TYPE_AREA;
            if (std::strcmp(landuse, "basin") == 0)     return WATER_TYPE_AREA;
        }
        const char* natural = tags["natural"];
        if (natural) {
            if (std::strcmp(natural, "water") == 0)   return WATER_TYPE_AREA;
            if (std::strcmp(natural, "glacier") == 0) return WATER_TYPE_AREA;
        }
        return -1;
    }

    // https://wiki.openstreetmap.org/wiki/Key=waterway
    int get_water_line_type(const osmium::TagList& tags) const
    {
        if (is_underground(tags)) {
            return -1;
        }
        const char* waterway = tags["waterway"];
        if (waterway)
        {
            const char* bridge = tags["bridge"];
            if (bridge) {
                if (std::strcmp(bridge, "yes") == 0)      return -1;
                if (std::strcmp(bridge, "aqueduct") == 0) return -1;
            }
            if (std::strcmp(waterway, "river") == 0)  return WATER_TYPE_LINE_RIVER;
            if (std::strcmp(waterway, "canal") == 0)  return WATER_TYPE_LINE_CANAL;
            if (std::strcmp(waterway, "stream") == 0) return WATER_TYPE_LINE_STREAM;            
            if (std::strcmp(waterway, "ditch") == 0)  return WATER_TYPE_LINE_DITCH;
            if (std::strcmp(waterway, "drain") == 0)  return WATER_TYPE_LINE_DRAIN;
        }
        return -1;
    }

    double get_water_line_width(const osmium::TagList& tags, water_type type) const
    {
        double width;
        if (parse_num(tags["width"], width)) {
            return width;
        }

        // pixel widths at zoom 17 from carto
        switch (type)
        {
            case WATER_TYPE_LINE_RIVER:  return 10.0;
            case WATER_TYPE_LINE_CANAL:  return 10.0;
            case WATER_TYPE_LINE_STREAM: return 3.0;
            case WATER_TYPE_LINE_DITCH:  return 2.0;
            case WATER_TYPE_LINE_DRAIN:  return 2.0;
            default: 
                assert(false); 
                return -1.0;
        }
        // scale to meters
        return width * (50.0 / 58.0);
    }
};

#endif