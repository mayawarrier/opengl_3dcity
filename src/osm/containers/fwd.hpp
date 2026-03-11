
#ifndef OSM_CONTAINERS_FWD_HPP
#define OSM_CONTAINERS_FWD_HPP

#include <memory>
#include <bitset>
#include "../common.hpp"

template <class Traits, int Dim, class TObj>
concept AabbTraits = requires(Traits traits, TObj obj) {
    typename Traits::search_flags_type;
    { traits.bbox(obj) } -> std::same_as<const bbox<Dim>&>;
    { traits.flags(obj) } -> std::same_as<const typename Traits::search_flags_type&>;
}
&& is_instance_of_nontype<typename Traits::search_flags_type, std::bitset>::value;

// By default, expect a pointer type. User can provide custom traits if needed.
template <class T>
using default_aabb_ptr_traits = typename std::pointer_traits<T>::element_type::aabb_traits;

template <int N, class T, class Traits = default_aabb_ptr_traits<T>>
    requires AabbTraits<Traits, N, T>
class aabb_tree;

template <class T, class Traits = default_aabb_ptr_traits<T>>
using aabb_tree2d = aabb_tree<2, T, Traits>;

template <class T, class Traits = default_aabb_ptr_traits<T>>
using aabb_tree3d = aabb_tree<3, T, Traits>;

template <class Traits, class TWay>
concept WayNetworkTraits = requires(Traits traits, const TWay* way) {
    typename Traits::way_enum_type;
    { Traits::way_type(way) } -> std::same_as<typename Traits::way_enum_type>;
};

template <class TWay, class Traits = typename TWay::way_net_traits>
    requires WayNetworkTraits<Traits, TWay>
struct way_network;

#endif