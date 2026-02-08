
#ifndef OSM_AABB_TREE_HPP
#define OSM_AABB_TREE_HPP

#include <functional>
#include <boost/container/small_vector.hpp>

#include "../common.hpp"
#include "../geom.hpp"
#include "fwd.hpp"


// Data returned from query_nearest().
// Extend this struct to add more data.
struct aabb_tree_qdata
{
    double sqdist;
};


// Axis-aligned bounding box tree.
// 
// - Static (can be built only once).
// - Does not own the objects stored. 
// - Object type should be a pointer or a cheap-to-copy
//   type that points to the actual data.
// - Calling Traits::bbox(obj) should be free (i.e. the 
//   object should store its own bbox).
//
template <int N, typename T, typename Traits>
class aabb_tree
{
private:
    struct node 
    {
        union udata {
            bbox<N> bbox;
            T obj;
        } data;
        int lt_idx;
        int rt_idx;
    };

    static_assert(requires {
        { Traits::bbox(std::declval<T>()) } -> std::same_as<const bbox<N>&>;
    }, "Invalid Traits type.");

public:
    aabb_tree() : 
        m_rootidx(-1) 
    {}

    DISABLE_COPY(aabb_tree)
    DEFAULT_MOVE(aabb_tree)

    // Changes the order of the source array!
    static aabb_tree create_unsafe(std::span<T> objects) {
        return { objects };
    }

    std::vector<T> query_bbox_all(const bbox<N>& query_bbox) const
    {
        std::vector<T> ret;

        // Good for upto 2^16 objects.
        types::small_vector<int, 16> search_nodeids;

        auto check_subtree = [&](int nidx) {
            if (has_nodeid(nidx) && query_bbox.intersects(node_bbox(m_nodes[nidx]))) {
                search_nodeids.push_back(nidx);
            }
        };

        check_subtree(m_rootidx);

        while (!search_nodeids.empty())
        {
            int nidx = search_nodeids.back();
            auto& node = m_nodes[nidx];
            search_nodeids.pop_back();

            check_subtree(node.lt_idx);
            check_subtree(node.rt_idx);

            if (is_leaf(node)) {
                ret.push_back(node.data.obj);
            }
        }
        return ret;
    }

    //
    // Get the nearest object in the tree to the query (ray or point).
    // Rays must be normalized.
    // \param obj_intersect_cb Callback fn to confirm intersection with object when query intersects its bbox.
    // \param out_qdata Additional data about the intersection/object.
    //
    template <class QueryData = aabb_tree_qdata>
    bool query_nearest(const auto& query, 
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
                intersects_subtree(query, node_bbox(m_nodes[nidx]), dist_range, sqdist) &&
                sqdist < best_qdata.sqdist) {
                search_nodeids.push_back(nidx);
            }
        };

        check_subtree(m_rootidx);
        
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
                bool inter = std::invoke(obj_intersect_cb, node.data.obj, qdata);
                if (inter && qdata.sqdist < best_qdata.sqdist) {
                    best_object = &node.data.obj;
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

private:
    bool intersects_subtree(const ray2d& ray, 
        const bbox<N>& subtree_bbox, const param_range& dist_range, double& out_sqdist) const
    {
        auto& bbox = subtree_bbox;
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

    bool intersects_subtree(const glm::dvec2& query, 
        const bbox<N>& subtree_bbox, const param_range& radius_range, double& out_sqdist) const
    {
        auto& bbox = subtree_bbox;
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

    aabb_tree(std::span<T> objects) {
        m_rootidx = make_tree(objects.data(), objects.size());
    }

    int make_tree(T* objects, size_t num_objects)
    {
        if (num_objects == 0) {
            return -1;
        }
        else if (num_objects == 1)
        {
            int node_idx = int(m_nodes.size());
            auto& node = m_nodes.emplace_back();

            node.data.obj = objects[0];
            node.lt_idx = -1;
            node.rt_idx = -1;
            return node_idx;
        }
        else {
            int node_idx = int(m_nodes.size());
            auto& node = m_nodes.emplace_back();
            auto& node_bb = node.data.bbox;

            node_bb = bbox<N>::empty();
            for (size_t i = 0; i < num_objects; ++i) {
                node_bb.extend(Traits::bbox(objects[i]));
            }

            auto dim_sizes = node_bb.max - node_bb.min;
            int longest_dim = vec_argmax(dim_sizes);

            // Divide objects into half along longest dimension
            std::sort(objects, objects + num_objects, [&](const T& lhs, const T& rhs) 
            {
                double lhs_dim = Traits::bbox(lhs).center()[longest_dim];
                double rhs_dim = Traits::bbox(rhs).center()[longest_dim];
                return lhs_dim < rhs_dim;
            });
            size_t lt_size = num_objects / 2; // truncates
            size_t rt_size = num_objects - lt_size;

            // Node reference is invalid after this point because subsequent
            // make_tree() calls can cause m_nodes to reallocate, so use node_idx
            m_nodes[node_idx].lt_idx = make_tree(objects, lt_size);
            m_nodes[node_idx].rt_idx = make_tree(objects + lt_size, rt_size);

            return node_idx;
        }
    }

    inline bool has_nodeid(int node_idx) const {
        return node_idx != -1;
    }

    inline bool is_leaf(const node& n) const {
        return !has_nodeid(n.lt_idx) && !has_nodeid(n.rt_idx);
    }

    inline const bbox<N>& node_bbox(const node& n) const {
        return is_leaf(n) ? Traits::bbox(n.data.obj) : n.data.bbox;
    }

private:
    std::vector<node> m_nodes;
    int m_rootidx; // -1 or 0
};

#endif