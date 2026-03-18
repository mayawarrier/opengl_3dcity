
#ifndef OSM_AABB_TREE_HPP
#define OSM_AABB_TREE_HPP

#include <functional>
#include <boost/container/small_vector.hpp>

#include "../common.hpp"
#include "../geom/geom.hpp"
#include "fwd.hpp"


// Data returned from query_nearest().
// Extend this struct to add more data.
struct aabb_tree_qdata
{
    double sqdist;
};

struct aabb_tree_unsafe_ctor_t {
    explicit aabb_tree_unsafe_ctor_t() = default;
};

// Axis-aligned bounding box tree.
// 
// - Static (can be built only once).
// - Does not own the objects stored. 
// - Object type should be a pointer or a cheap-to-copy
//   POD type that points to the actual data.
// - Calling Traits::bbox(obj) should be free
//   (ideally the object should store its own bbox).
//
template <int N, class T, class Traits>
    requires AabbTraits<Traits, N, T>
class aabb_tree
{
public:
    using search_flags_type = typename Traits::search_flags_type;

private:
    struct node 
    {
        struct internal_t
        {
            bbox<N> bbox;
            // it's okay if this has a non-trivial dtor, since
            // the active member will never be switched.
            search_flags_type flags;
        };
        struct leaf_t {
            T obj;
        };
        union data_t
        {
            internal_t internal;
            leaf_t leaf;
        };
        data_t data;
        int lt_idx;
        int rt_idx;
    };

public:
    aabb_tree() : 
        m_data(ebco_second_args_t{}, -1)
    {}

    // Changes the order of the source array!
    aabb_tree(aabb_tree_unsafe_ctor_t, std::span<T> objects, const Traits& traits = Traits()) :
        aabb_tree(objects, traits)
    {}

    // Changes the order of the source array!
    aabb_tree(aabb_tree_unsafe_ctor_t, std::span<T> objects, Traits&& traits) :
        aabb_tree(objects, std::move(traits))
    {}

    aabb_tree(std::span<const T> objects, const Traits& traits = Traits()) :
        aabb_tree(copy_objects_t{}, objects, traits)
    {}

    aabb_tree(std::span<const T> objects, Traits&& traits) :
        aabb_tree(copy_objects_t{}, objects, std::move(traits))
    {}

    DISABLE_COPY(aabb_tree)
    DEFAULT_MOVE(aabb_tree)

    std::vector<T> query_intersecting_bboxes(
        const bbox<N>& query_bbox, const search_flags_type& query_flags) const
    {
        std::vector<T> ret;

        // Good for upto 2^16 objects.
        types::small_vector<int, 16> search_nodeids;

        auto check_subtree = [&](int nidx) 
        {
            if (has_nodeid(nidx) && 
                (query_flags & node_flags(m_nodes[nidx])).any() && 
                query_bbox.intersects(node_bbox(m_nodes[nidx]))) 
            {
                search_nodeids.push_back(nidx);
            }
        };

        check_subtree(rootidx_ref());

        while (!search_nodeids.empty())
        {
            int nidx = search_nodeids.back();
            auto& node = m_nodes[nidx];
            search_nodeids.pop_back();

            check_subtree(node.lt_idx);
            check_subtree(node.rt_idx);

            if (is_leaf(node)) {
                ret.push_back(node.data.leaf.obj);
            }
        }
        return ret;
    }

    std::vector<T> query_intersecting_bboxes(const bbox<N>& query_bbox) const
    {
        search_flags_type all_flags;
        all_flags.set(); // all bits 1
        return query_intersecting_bboxes(query_bbox, all_flags);
    }

    //
    // Get the nearest object in the tree to the query (ray or point).
    // Rays must be normalized.
    // \param obj_intersect_cb Callback fn to confirm intersection with object when query intersects its bbox.
    // \param out_qdata Additional data about the intersection/object.
    //
    template <class QueryData = aabb_tree_qdata>
    bool query_nearest(const auto& query, const search_flags_type& query_flags,
        const param_range& dist_range, auto obj_intersect_cb, T& out_object, QueryData& out_qdata) const
    {
        static_assert(requires {
            { std::invoke(obj_intersect_cb, std::declval<const T&>(), std::declval<QueryData&>()) } -> std::same_as<bool>;
        }, "Invalid obj_intersect callback.");

        const T* best_object = nullptr;
        QueryData best_qdata{};
        best_qdata.sqdist = std::numeric_limits<double>::infinity();

        // Good for upto 2^16 objects.
        types::small_vector<int, 16> search_nodeids;

        auto check_subtree = [&](int nidx) 
        {
            double sqdist = std::numeric_limits<double>::infinity();
            if (has_nodeid(nidx) && 
                (query_flags & node_flags(m_nodes[nidx])).any() &&
                query_intersects_bbox(query, dist_range, node_bbox(m_nodes[nidx]), sqdist) &&
                sqdist < best_qdata.sqdist) 
            {
                search_nodeids.push_back(nidx);
            }
        };

        check_subtree(rootidx_ref());
        
        while (!search_nodeids.empty())
        {
            int nidx = search_nodeids.back();
            auto& node = m_nodes[nidx];
            search_nodeids.pop_back();

            check_subtree(node.lt_idx);
            check_subtree(node.rt_idx);

            if (is_leaf(node))
            {
                QueryData qdata{};
                bool inter = std::invoke(obj_intersect_cb, node.data.leaf.obj, qdata);
                if (inter && qdata.sqdist < best_qdata.sqdist) {
                    best_object = &node.data.leaf.obj;
                    best_qdata = std::move(qdata);
                }
            }
        }
        if (best_object) {
            out_object = *best_object;
            out_qdata = std::move(best_qdata);
            return true;
        }
        else { return false; }
    }

    template <class QueryData = aabb_tree_qdata>
    bool query_nearest(const auto& query,
        const param_range& dist_range, auto obj_intersect_cb, T& out_object, QueryData& out_qdata) const
    {
        search_flags_type all_flags;
        all_flags.set(); // all bits 1
        return query_nearest(query, all_flags, dist_range, obj_intersect_cb, out_object, out_qdata);
    }

private:
    struct copy_objects_t {
        explicit copy_objects_t() = default;
    };

    template <class FwTraits>
    aabb_tree(std::span<T> objects, FwTraits&& traits) :
        m_data(ebco_first_then_second_args_t{}, std::forward<FwTraits>(traits), -1)
    {
        init(objects);
    }

    template <class FwTraits>
    aabb_tree(copy_objects_t, std::span<const T> objects, FwTraits&& traits) :
        m_data(ebco_first_then_second_args_t{}, std::forward<FwTraits>(traits), -1)
    {
        std::vector<T> copied_objects(objects.begin(), objects.end());
        init(copied_objects);
    }

    void init(std::span<T> objects) {
        rootidx_ref() = make_tree(objects.data(), objects.size());
    }

    int make_tree(T* objects, size_t num_objects)
    {
        if (num_objects == 0) {
            return -1;
        }
        else if (num_objects == 1)
        {
            int node_idx = int(m_nodes.size());

            typename node::leaf_t leaf{
                .obj = objects[0]
            };
            m_nodes.push_back(node{
                .data = { .leaf = leaf },
                .lt_idx = -1,
                .rt_idx = -1
            });
            return node_idx;
        }
        else {
            typename node::internal_t data{};

            for (size_t i = 0; i < num_objects; ++i) {
                data.bbox.extend(traits().bbox(objects[i]));
                data.flags |= traits().flags(objects[i]);
            }

            auto dim_sizes = data.bbox.max - data.bbox.min;
            int longest_dim = vec_argmax(dim_sizes);

            // Divide objects into half along longest dimension
            std::sort(objects, objects + num_objects, [&](const T& lhs, const T& rhs) 
            {
                double lhs_dim = traits().bbox(lhs).center()[longest_dim];
                double rhs_dim = traits().bbox(rhs).center()[longest_dim];
                return lhs_dim < rhs_dim;
            });
            size_t lt_size = num_objects / 2; // truncates
            size_t rt_size = num_objects - lt_size;

            int node_idx = int(m_nodes.size());
            m_nodes.push_back(node{
                .data = { .internal = data },
                .lt_idx = -1,
                .rt_idx = -1
            });

            // Node reference is invalid after this point because subsequent
            // make_tree() calls can cause m_nodes to reallocate, so use node_idx     
            m_nodes[node_idx].lt_idx = make_tree(objects, lt_size);
            m_nodes[node_idx].rt_idx = make_tree(objects + lt_size, rt_size);

            return node_idx;
        }
    }

    bool query_intersects_bbox(const ray<N>& ray, 
        const param_range& dist_range, const bbox<N>& bbox, double& out_sqdist) const
    {
        double dentry = -std::numeric_limits<double>::infinity();
        double dexit = std::numeric_limits<double>::infinity();

        // intersect ray with every slab of the box
        for (int i = 0; i < N; ++i)
        {
            if (ray.dir[i] == 0) [[unlikely]] {
                // if not moving in this dim, must be within bounds
                if (ray.origin[i] < bbox.min[i] || ray.origin[i] > bbox.max[i]) {
                    return false;
                }
                continue;
            }
            // intersections with min and max planes
            double dmin = (bbox.min[i] - ray.origin[i]) / ray.dir[i];
            double dmax = (bbox.max[i] - ray.origin[i]) / ray.dir[i];

            auto [d1, d2] = std::minmax(dmin, dmax);
            dentry = std::max(dentry, d1);
            dexit = std::min(dexit, d2);
        }

        if (dentry > dexit || !dist_range.overlaps(dentry, dexit)) {
            return false;
        }
        // dentry <= dist_range.max already checked
        double d = std::max(dentry, dist_range.min());
        out_sqdist = d * d;
        return true;
    }

    bool query_intersects_bbox(const glm::vec<N, double>& query, 
        const param_range& radius_range, const bbox<N>& bbox, double& out_sqdist) const
    {
        assert(radius_range.nonneg());

        double dmin2 = 0.0, dmax2 = 0.0;
        for (int i = 0; i < N; ++i) 
        {
            // If within the slab in this dim, no contribution
            // If all dims are inside, then dmin2 = 0
            double dmin = 0.0;
            if (query[i] < bbox.min[i]) {
                dmin = (bbox.min[i] - query[i]);
            } 
            else if (query[i] > bbox.max[i]) {
                dmin = (query[i] - bbox.max[i]);
            }
            dmin2 += dmin * dmin;

            // Furthest point is always one of the corners
            // Add the max distance to a slab plane in this dim
            double dmax = std::max(
                std::abs(bbox.min[i] - query[i]), 
                std::abs(bbox.max[i] - query[i]));
            dmax2 += dmax * dmax;
        }

        if (!radius_range.overlaps2(dmin2, dmax2)) {
            return false;
        }
        out_sqdist = std::max(dmin2, radius_range.min2());
        return true;
    }

    inline bool has_nodeid(int node_idx) const {
        return node_idx != -1;
    }

    inline bool is_leaf(const node& n) const {
        return !has_nodeid(n.lt_idx) && !has_nodeid(n.rt_idx);
    }

    inline const bbox<N>& node_bbox(const node& n) const {
        return is_leaf(n) ? traits().bbox(n.data.leaf.obj) : n.data.internal.bbox;
    }

    inline const search_flags_type& node_flags(const node& n) const {
        return is_leaf(n) ? traits().flags(n.data.leaf.obj) : n.data.internal.flags;
    }

    inline Traits& traits() noexcept { return m_data.ebco_first(); }
    inline const Traits& traits() const noexcept { return m_data.ebco_first(); }

    // Either -1 or 0.
    inline int& rootidx_ref() noexcept { return m_data.ebco_second(); }
    inline const int& rootidx_ref() const noexcept { return m_data.ebco_second(); }

private:
    std::vector<node> m_nodes;
    ebco_pair<Traits, int> m_data;
};

#endif