
#ifndef OSM_MESH_COMP_BUILDERS_TYPES_HPP
#define OSM_MESH_COMP_BUILDERS_TYPES_HPP

#include <vector>
#include <bitset>

#include <osmium/osm/object.hpp>

#include "../common.hpp"
#include "../containers/aabb_tree.hpp"
#include "../containers/way_network.hpp"


enum ent_comp_type
{
    COMP_TYPE_HIGHWAY,
    COMP_TYPE_WATER,
    COMP_TYPE_BUILDING,
    COMP_TYPE_BUILDING_PART,
    COMP_NUM_TYPES // keep this last
};

enum ent_way_comp_type
{
    WAY_COMP_TYPE_HIGHWAY,
    WAY_COMP_TYPE_WATERWAY,
    WAY_COMP_NUM_TYPES
};

struct mesh_entity
{
    struct comp_info
    {
        ent_comp_type type;
        int comp_idx;
    };
    using comp_flags_t = std::bitset<COMP_NUM_TYPES>;
    using way_comp_flags_t = std::bitset<WAY_COMP_NUM_TYPES>;
    using comp_info_vec_t = types::small_vector<comp_info, 1>;

    osm_obj_type obj_type;
    int obj_idx;
    bbox2d bbox;
    comp_flags_t comp_flags;
    way_comp_flags_t way_comp_flags;
    comp_info_vec_t comps;
    std::string name;

    struct aabb_traits
    {
        using query_flags_t = mesh_entity::comp_flags_t;

        static const bbox2d& bbox(const mesh_entity* ent) {
            return ent->bbox;
        }
        static const query_flags_t& flags(const mesh_entity* ent) {
            return ent->comp_flags;
        }
    };

    struct way_net_traits
    {
        using way_type_t = ent_way_comp_type;
        using way_type_flags_t = mesh_entity::way_comp_flags_t;

        static const way_type_flags_t& type_flags(const mesh_entity* ent) {
            return ent->way_comp_flags;
        }
    };
};

struct mesh_entity_db
{
    glm::dvec2 center;
    // Scale heights by this factor to account for 
    // distortion from mercator projection.
    double ht_scale;

    std::vector<osm_node> nodes;
    std::vector<osm_way> ways;
    std::vector<osm_area> areas;
    std::vector<mesh_entity> entities;
    aabb_tree2d<const mesh_entity*> entity_tree;
    way_network<mesh_entity> way_net;

    template <class Obj>
    const Obj& get(int idx) const
    {
        if constexpr (std::same_as<Obj, osm_node>) {
            return nodes[idx];
        } else if constexpr (std::same_as<Obj, osm_way>) {
            return ways[idx];
        } else if constexpr (std::same_as<Obj, osm_area>) {
            return areas[idx];
        } else if constexpr (std::same_as<Obj, mesh_entity>) {
            return entities[idx];
        } else {
            static_assert(deferred_false_v<Obj>, "Unsupported object type");
        }
    }

    osmium::object_id_type ent_osm_id(const mesh_entity& ent) const 
    {
        switch (ent.obj_type) {
            case OSM_OBJ_TYPE_NODE:
                return nodes[ent.obj_idx].id;
            case OSM_OBJ_TYPE_WAY:
                return ways[ent.obj_idx].id;
            case OSM_OBJ_TYPE_AREA:
                return areas[ent.obj_idx].id;
            default:
                assert_msg(false, "Invalid OSM object type");
                return -1;
        }
    }

    int ent_comp_idx(const mesh_entity& ent, ent_comp_type type) const
    {
        assert(!ent.comps.empty());
        if (ent.comps[0].type != type) [[ unlikely ]] {
            auto icomp_it = std::ranges::find_if(ent.comps, [&](const auto& ci) {
                return ci.type == type;
            });
            return icomp_it != ent.comps.end() ? icomp_it->comp_idx : -1;
        }
        return ent.comps[0].comp_idx;
    }
};

template <class... TComps>
struct mesh_comp_db;

template <class T, class... TComps>
concept MeshCompBuilder = requires(
    T builder,
    int entity_idx,
    const osmium::OSMObject* obj,
    mesh_comp_db<TComps...>* comp_db,
    mesh_entity::comp_info_vec_t& comps,
    const mesh_entity_db* entity_db,
    std::vector<osm_tri_datad>& out_tridata)
{
    typename T::comp_t;
    { builder.comp_type_name() } -> std::convertible_to<const char*>;
    { builder.add_comp(entity_idx, obj, comp_db, comps) } -> std::same_as<bool>;
    { builder.build_all(entity_db, comp_db, out_tridata) } -> std::same_as<bool>;
};

// Each comp builder gets access to all comps for all objects, 
// for eg. the street builder needs to avoid generating streets
// that intersect buildings.
template <class... TComps>
struct mesh_comp_db
{
    std::tuple<std::vector<TComps>...> comps;

    template <class TComp>
    std::vector<TComp>& comps_vec()
    {
        return std::get<std::vector<TComp>>(comps);
    }
};

#endif