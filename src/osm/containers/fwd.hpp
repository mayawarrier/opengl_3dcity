
#ifndef OSM_CONTAINERS_FWD_HPP
#define OSM_CONTAINERS_FWD_HPP

#include <memory>

template <typename T>
using aabb_traits = typename std::pointer_traits<T>::element_type::aabb_traits;

template <int N, typename T, typename Traits = aabb_traits<T>>
class aabb_tree;

template <typename TWay, typename Traits = typename TWay::way_net_traits>
struct way_network;

template <typename T, typename Traits = aabb_traits<T>>
using aabb_tree2d = aabb_tree<2, T, Traits>;

template <typename T, typename Traits = aabb_traits<T>>
using aabb_tree3d = aabb_tree<3, T, Traits>;

#endif