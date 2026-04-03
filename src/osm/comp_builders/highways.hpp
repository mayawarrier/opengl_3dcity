
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
    int entity_idx;
    highway_type type;
    double width;
};

struct highway_comp_builder
{
    using comp_t = highway_comp;
    constexpr const char* comp_type_name() const { return "highway"; }

    template <class ...TComps>
    bool add_comp(int entity_idx, const osmium::OSMObject* obj,
        mesh_comp_db<TComps...>* comp_db, mesh_entity::comp_info_vec_t& comps_info)
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
                    .entity_idx = entity_idx,
                    .type = htype,
                    .width = get_highway_width(*way, htype)
                });
                comps_info.push_back({
                    .type = COMP_TYPE_HIGHWAY,
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
    bool build_all(const mesh_entity_db* obj_db,
        mesh_comp_db<TComps...>* comp_db, std::vector<osm_tri_datad>& out_drawdata)
    {
        auto& streets = comp_db->comps_vec<highway_comp>();
        auto& bldgs = comp_db->comps_vec<building_comp>();
        return do_build_all(obj_db, streets, bldgs, out_drawdata);
    }

private:
    bool do_build_all(const mesh_entity_db* obj_db, const std::vector<highway_comp>& highways,
        const std::vector<building_comp>& buildings, std::vector<osm_tri_datad>& out_drawdata);

    // Type of highway based on its tags. 
    // Returns -1 if not a highway and HIGHWAY_TYPE_UNKNOWN if 
    // the highway tag is present but unrecognized.
    static int get_highway_type(const osmium::Way& way)
    {
        const char* highway = way.tags()["highway"];
        if (!highway) {
            return -1;
        }
        if (std::strcmp(highway, "motorway") == 0)       return HIGHWAY_TYPE_MOTORWAY;
        if (std::strcmp(highway, "trunk") == 0)          return HIGHWAY_TYPE_TRUNK;
        if (std::strcmp(highway, "primary") == 0)        return HIGHWAY_TYPE_PRIMARY;
        if (std::strcmp(highway, "secondary") == 0)      return HIGHWAY_TYPE_SECONDARY;
        if (std::strcmp(highway, "tertiary") == 0)       return HIGHWAY_TYPE_TERTIARY;
        if (std::strcmp(highway, "unclassified") == 0)   return HIGHWAY_TYPE_UNCLASSIFIED;
        if (std::strcmp(highway, "residential") == 0)    return HIGHWAY_TYPE_RESIDENTIAL;
        if (std::strcmp(highway, "motorway_link") == 0)  return HIGHWAY_TYPE_MOTORWAY_LINK;
        if (std::strcmp(highway, "trunk_link") == 0)     return HIGHWAY_TYPE_TRUNK_LINK;
        if (std::strcmp(highway, "primary_link") == 0)   return HIGHWAY_TYPE_PRIMARY_LINK;
        if (std::strcmp(highway, "secondary_link") == 0) return HIGHWAY_TYPE_SECONDARY_LINK;
        if (std::strcmp(highway, "tertiary_link") == 0)  return HIGHWAY_TYPE_TERTIARY_LINK;
        if (std::strcmp(highway, "living_street") == 0)  return HIGHWAY_TYPE_LIVING_STREET;
        if (std::strcmp(highway, "service") == 0)        return HIGHWAY_TYPE_SERVICE;
        if (std::strcmp(highway, "pedestrian") == 0)     return HIGHWAY_TYPE_PEDESTRIAN;
        if (std::strcmp(highway, "road") == 0)           return HIGHWAY_TYPE_ROAD;

        if (std::strcmp(highway, "footway") == 0)
        {
            const char* footway = way.tags()["footway"];
            if (footway && std::strcmp(footway, "sidewalk") == 0) return HIGHWAY_TYPE_FOOTWAY_SIDEWALK;
            if (footway && std::strcmp(footway, "crossing") == 0) return HIGHWAY_TYPE_FOOTWAY_CROSSING;
            return HIGHWAY_TYPE_FOOTWAY;
        }

        return HIGHWAY_TYPE_UNKNOWN;
    }

    // Width of a highway type in meters. Uses the width tag if present.
    // Returns -1 if type is invalid or for highways with tag area=yes.
    static double get_highway_width(const osmium::Way& way, highway_type type)
    {
        if (way.tags().has_tag("area", "yes")) {
            return -1.0;
        }

        double width;
        switch (type)
        {
        case HIGHWAY_TYPE_MOTORWAY:         width = 18.0;  break;
        case HIGHWAY_TYPE_TRUNK:            width = 18.0;  break;
        case HIGHWAY_TYPE_PRIMARY:          width = 18.0;  break;
        case HIGHWAY_TYPE_SECONDARY:        width = 18.0;  break;
        case HIGHWAY_TYPE_TERTIARY:         width = 18.0;  break;
        case HIGHWAY_TYPE_MOTORWAY_LINK:    width = 12.0;  break;
        case HIGHWAY_TYPE_TRUNK_LINK:       width = 12.0;  break;
        case HIGHWAY_TYPE_PRIMARY_LINK:     width = 12.0;  break;
        case HIGHWAY_TYPE_SECONDARY_LINK:   width = 12.0;  break;
        case HIGHWAY_TYPE_TERTIARY_LINK:    width = 12.0;  break;
        case HIGHWAY_TYPE_UNCLASSIFIED:     width = 12.0;  break;
        case HIGHWAY_TYPE_RESIDENTIAL:      width = 12.0;  break;
        case HIGHWAY_TYPE_LIVING_STREET:    width = 12.0;  break;
        case HIGHWAY_TYPE_PEDESTRIAN:       width = 12.0;  break;
        case HIGHWAY_TYPE_SERVICE:          width = 7.0;   break;
        case HIGHWAY_TYPE_FOOTWAY:
        case HIGHWAY_TYPE_FOOTWAY_SIDEWALK:
        case HIGHWAY_TYPE_FOOTWAY_CROSSING: width = 1.3;   break;
        case HIGHWAY_TYPE_UNKNOWN:
        case HIGHWAY_TYPE_ROAD:             width = 7.0;   break;
        default:
            assert(false);
            return -1.0;
        }
        // roughly convert from carto's pixel-based widths
        return width * (50.0 / 58.0);
    }

private:
    int m_num_hiway_nodes = 0;
};

#endif