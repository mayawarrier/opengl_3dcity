
#ifndef OSM_MESH_COMP_BUILDERS_TYPES_HPP
#define OSM_MESH_COMP_BUILDERS_TYPES_HPP

#include <vector>
#include <bitset>

#include <osmium/osm/object.hpp>

#include "../common.hpp"
#include "../containers/aabb_tree.hpp"


enum osm_comp_type
{
    COMP_TYPE_STREET,
    COMP_TYPE_BUILDING,
    COMP_TYPE_BUILDING_PART,
    COMP_NUM_TYPES // keep this last
};

struct osm_mesh_object
{
    struct comp_info
    {
        osm_comp_type type;
        int comp_idx;
    };
    using comp_flags_t = std::bitset<COMP_NUM_TYPES>;
    using comp_info_vec_t = types::small_vector<comp_info, 1>;

    osm_obj_type type;
    int osm_obj_idx;
    bbox2d bbox;
    comp_flags_t comp_flags;
    comp_info_vec_t comps; // Comp types must not repeat.
    std::string name;

    int get_comp_idx(osm_comp_type type) const 
    {
        if (comps.empty() || comps[0].type != type) [[ unlikely ]]
        {
            auto icomp_it = std::ranges::find_if(comps, [&](const auto& ci) {
                return ci.type == type;
            });
            return icomp_it != comps.end() ? icomp_it->comp_idx : -1;
        }
        return comps[0].comp_idx;
    }

    struct aabb_traits
    {
        using search_flags_type = osm_mesh_object::comp_flags_t;

        static const bbox2d& bbox(const osm_mesh_object* obj) {
            return obj->bbox;
        }
        static const search_flags_type& flags(const osm_mesh_object* obj) {
            return obj->comp_flags;
        }
    };
};

struct osm_mesh_object_db
{
    bbox2d objs_bbox;
    std::vector<osm_node> nodes;
    std::vector<osm_way> ways;
    std::vector<osm_area> areas;
    std::vector<osm_mesh_object> objects;
    aabb_tree2d<const osm_mesh_object*> obj_tree;

    template <class Obj>
    const Obj& get(int idx) const
    {
        if constexpr (std::same_as<Obj, osm_node>) {
            return nodes[idx];
        } else if constexpr (std::same_as<Obj, osm_way>) {
            return ways[idx];
        } else if constexpr (std::same_as<Obj, osm_area>) {
            return areas[idx];
        } else if constexpr (std::same_as<Obj, osm_mesh_object>) {
            return objects[idx];
        } else {
            static_assert(deferred_false_v<Obj>, "Unsupported object type");
        }
    }

    osmium::object_id_type obj_osm_id(const osm_mesh_object& mesh_obj) const {
        switch (mesh_obj.type) {
            case OSM_OBJ_TYPE_NODE:
                return nodes[mesh_obj.osm_obj_idx].id;
            case OSM_OBJ_TYPE_WAY:
                return ways[mesh_obj.osm_obj_idx].id;
            case OSM_OBJ_TYPE_AREA:
                return areas[mesh_obj.osm_obj_idx].id;
            default:
                assert_msg(false, "Invalid OSM object type");
                return -1;
        }
    }
};

template <class... TComps>
struct osm_mesh_comp_db;

template <class T, class... TComps>
concept MeshCompBuilder = requires(
    T builder,
    int mesh_obj_idx,
    const osmium::OSMObject* obj,
    osm_mesh_comp_db<TComps...>* comp_db,
    osm_mesh_object::comp_info_vec_t& comps,
    const osm_mesh_object_db* obj_db,
    std::vector<draw_datad>& out_drawdata)
{
    typename T::comp_type;
    { builder.comp_type_name() } -> std::convertible_to<const char*>;
    { builder.add_comp(mesh_obj_idx, obj, comp_db, comps) } -> std::same_as<bool>;
    { builder.build_all(obj_db, comp_db, out_drawdata) } -> std::same_as<bool>;
};

// Each comp builder gets access to all comps for all objects, 
// for eg. the street builder needs to avoid generating streets
// that intersect buildings.
template <class... TComps>
struct osm_mesh_comp_db
{
    std::tuple<std::vector<TComps>...> comps;

    template <class TComp>
    std::vector<TComp>& comps_vec()
    {
        return std::get<std::vector<TComp>>(comps);
    }
};

#endif