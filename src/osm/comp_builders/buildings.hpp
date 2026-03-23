
#ifndef OSM_MESH_BUILDINGS_HPP
#define OSM_MESH_BUILDINGS_HPP

#include "types.hpp"


// todo: add more types
// https://wiki.openstreetmap.org/wiki/Buildings
enum building_type
{
    BUILDING_TYPE_YES
};

// todo: add more types
// https://wiki.openstreetmap.org/wiki/Key:building:part
enum building_part_type
{
    BUILDING_PART_TYPE_YES,

    // Illegal type. bldg_comp.type will never be set to this.
    // Mappers in rare cases add building:part=no in combination
    // with building=yes to indicate that the building must 
    // only be rendered as its parts.
    BUILDING_PART_TYPE_NO 
};

struct building_comp
{
    int mesh_obj_idx;

    // building_type or building_part_type, 
    // depending on is_part
    int type; 

    bool is_part;
    bool parts_only; // building=yes + building:part=no
    bool has_ht_btm, has_ht_top;
    
    // ht_btm and ht_top are set to default values of
    // 0.0 and 3.0 respectively if the corresponding 
    // has_ht_flag is false.
    double ht_btm, ht_top;
    double roof_ht;

    double overlap(const building_comp& other) const
    {
        double overlap_btm = std::max(ht_btm, other.ht_btm);
        double overlap_top = std::min(ht_top, other.ht_top);
        return std::max(0.0, overlap_top - overlap_btm);
    }
};

struct bldg_comp_builder
{
    using comp_type = building_comp;
    constexpr const char* comp_type_name() const { return "building"; }

    template <class ...TComps>
    bool add_comp(int mesh_obj_idx, const osmium::OSMObject* obj,
        osm_mesh_comp_db<TComps...>* comp_db, osm_mesh_object::comp_info_vec_t& comp_info)
    {
        if (obj->type() != osmium::item_type::area) {
            return false;
        }
        const auto* area = static_cast<const osmium::Area*>(obj);

        auto& tags = obj->tags();
        int bldg_type = get_building_type(tags);
        int bldg_part_type = get_building_part_type(tags);

        if (bldg_type != -1 || bldg_part_type != -1)
        {
            // prefer bldg_type over bldg_part_type as authoritative
            // since objects might have both building and building:part tags
            // (see note for BUILDING_PART_TYPE_NO)
            osm_comp_type comp_type = bldg_type != -1 ? COMP_TYPE_BUILDING : COMP_TYPE_BUILDING_PART;

            if (bldg_part_type == BUILDING_PART_TYPE_NO) 
            {
                if (comp_type != COMP_TYPE_BUILDING) {
                    return false;
                }
                logDEBUG(LOG_MESSAGE, "%s %lld has building=yes and building:part=no", 
                    area->from_way() ? "Way" : "Relation", area->orig_id());
            }

            auto& comps_vec = comp_db->comps_vec<building_comp>();
            building_comp comp = {
                .mesh_obj_idx = mesh_obj_idx,
                .type = comp_type == COMP_TYPE_BUILDING ? bldg_type : bldg_part_type,
                .is_part = comp_type == COMP_TYPE_BUILDING_PART,            
                .parts_only = bldg_part_type == BUILDING_PART_TYPE_NO
            };
            set_bldg_heights(tags, comp);

            comps_vec.push_back(comp);

            comp_info.push_back({
                .type = comp_type,
                .comp_idx = int(comps_vec.size()) - 1
            });
            return true;
        }

        return false;
    }

    template <class ...TComps>
    bool build_all(const osm_mesh_object_db* obj_db,
        osm_mesh_comp_db<TComps...>* comp_db, std::vector<draw_datad>& out_drawdata)
    {
        auto& bldgs = comp_db->comps_vec<building_comp>();
        return do_build_all(obj_db, bldgs, out_drawdata);
    }

private:
    bool do_build_all(const osm_mesh_object_db* obj_db, 
        const std::vector<building_comp>& bldgs, std::vector<draw_datad>& out_drawdata);

    int get_building_type(const osmium::TagList& tags) const
    {
        if (tags.has_key("building")) {
            return BUILDING_TYPE_YES;
        }
        return -1;
    }
    int get_building_part_type(const osmium::TagList& tags) const
    {
        const char* bldg_part = tags["building:part"];
        if (!bldg_part) {
            return -1;
        }
        if (std::strcmp(bldg_part, "no") == 0) return BUILDING_PART_TYPE_NO;
        return BUILDING_PART_TYPE_YES;
    }

    void set_bldg_heights(const osmium::TagList& tags, building_comp& comp);
};


#endif