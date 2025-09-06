
#ifndef OSM_CONTAINERS_FWD_HPP
#define OSM_CONTAINERS_FWD_HPP

#include <memory>

template <typename T, typename Traits = std::pointer_traits<T>::element_type::aabb_traits>
class aabb_tree;

template <typename TWay, typename Traits = TWay::way_net_traits>
struct way_network;

#endif