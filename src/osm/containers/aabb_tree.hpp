
#ifndef OSM_AABB_TREE_HPP
#define OSM_AABB_TREE_HPP

#include <functional>
#include <boost/container/small_vector.hpp>

#include "../common.hpp"
#include "../geom.hpp"
#include "fwd.hpp"


// Default data returned from nearest query.
struct aabb_querydata
{
    double sqdist;
};

//
// Axis-aligned bounding box tree.
// Does not own the objects stored. It is expected that the object type 
// is a pointer or a cheap-to-copy type that points to the actual data.
//
template <int N, typename T, typename Traits>
class aabb_tree
{
private:
    struct node 
    {
        node* left;
        node* right;
        bbox<N> bbox;
    };
    struct leafnode : public node {
        T data;
    };

    static_assert(requires {
        { Traits::bbox(std::declval<T>()) } -> std::same_as<const bbox<N>&>;
    }, "Invalid Traits type.");

public:
    aabb_tree() :
        m_root(nullptr)
    {}

    MOVE_ONLY_CLASS(aabb_tree, m_root, nullptr)

    // Changes the order of the source array!
    static aabb_tree create_unsafe(std::span<T> objects) {
        return { objects };
    }

    std::vector<T> query_bbox_all(const bbox<N>& bbox) const
    {
        std::vector<T> ret;
        // Good for trees with upto 2^16 objects.
        types::small_vector<const node*, 16> nodes;

        auto check_subtree = [&](const node* node) {
            if (node && bbox_intersects_bbox(node->bbox, bbox))
                nodes.push_back(node);
        };

        check_subtree(m_root);

        while (!nodes.empty())
        {
            auto* node = nodes.back();
            nodes.pop_back();

            check_subtree(node->left);
            check_subtree(node->right);

            if (!node->left && !node->right) {
                auto* leaf = (const leafnode*)node;
                ret.push_back(leaf->data);
            }
        }
        return ret;
    }

    //
    // Get the nearest object in the tree to the query (ray or point).
    // Rays must be normalized.
    // \param obj_intersect_cb Callback fn to confirm intersection with object.
    // \param out_qdata Additional data about the intersection/object.
    //
    template <typename QueryData = aabb_querydata>
    bool query_nearest(const auto& query, const param_range& dist_range, 
        auto obj_intersect_cb, T& out_object, QueryData& out_qdata) const
    {
        static_assert(requires {
            { std::invoke(obj_intersect_cb, 
                std::declval<const T&>(), std::declval<QueryData&>()) } -> std::same_as<bool>;
        }, "Invalid obj_intersect callback.");

        const T* best_object = nullptr;
        QueryData best_qdata{};
        best_qdata.sqdist = std::numeric_limits<double>::infinity();

        // Good for trees with upto 2^16 objects.
        types::small_vector<const node*, 16> nodes;

        auto check_subtree = [&](const node* node) 
        {
            double sqdist = std::numeric_limits<double>::infinity();
            if (node && intersects_subtree(node, query, dist_range, sqdist) && sqdist < best_qdata.sqdist)
                nodes.push_back(node);
        };

        check_subtree(m_root);
        
        while (!nodes.empty())
        {
            auto* node = nodes.back();
            nodes.pop_back();

            check_subtree(node->left);
            check_subtree(node->right);

            if (!node->left && !node->right) 
            {
                auto* leaf = (const leafnode*)node;

                QueryData qdata{};
                bool inter = std::invoke(obj_intersect_cb, leaf->data, qdata);
                if (inter && qdata.sqdist < best_qdata.sqdist) {
                    best_object = &leaf->data;
                    best_qdata = qdata;
                }
            }
        }
        if (best_object) {
            out_object = *best_object;
            out_qdata = best_qdata;
            return true;
        }
        else { return false; }
    }


    ~aabb_tree() {
        delete_tree(m_root);
    }

private:
    bool intersects_subtree(const node* node, const ray2d& ray, 
        const param_range& dist_range, double& out_sqdist) const
    {
        auto& bbox = node->bbox;

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

    bool intersects_subtree(const node* node, const glm::dvec2& query, 
        const param_range& radius_range, double& out_sqdist) const
    {
        auto& bbox = node->bbox;
        assert(radius_range.nonneg());

        // Closest and furthest points from query to bbox
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

    aabb_tree(std::span<T> objects) :
        m_root(make_tree(objects.data(), objects.size()))
    {}

    node* make_tree(T* objects, size_t num_objects)
    {
        if (num_objects == 0) {
            return nullptr;
        }
        else if (num_objects == 1)
        {
            auto* node = new leafnode();
            node->left = nullptr;
            node->right = nullptr;
            node->bbox = Traits::bbox(objects[0]);
            node->data = objects[0];
            return node;
        }
        else {
            auto* n = new node();

            for (size_t i = 0; i < num_objects; ++i) {
                n->bbox.extend(Traits::bbox(objects[i]));
            }

            auto dim_sizes = n->bbox.max - n->bbox.min;
            int longest_dim = vec_argmax(dim_sizes);

            std::sort(objects, objects + num_objects,
                [&](const T& lhs, const T& rhs) 
                {
                    double lhs_dim = Traits::bbox(lhs).center()[longest_dim];
                    double rhs_dim = Traits::bbox(rhs).center()[longest_dim];
                    return lhs_dim < rhs_dim;
                });

            size_t lhs_size = num_objects / 2; // truncated
            size_t rhs_size = num_objects - lhs_size;

            // Divide objects into half along longest dimension
            n->left = make_tree(objects, lhs_size);
            n->right = make_tree(objects + lhs_size, rhs_size);
            return n;
        }
    }

    void delete_tree(node* node)
    {
        if (!node) { return; }

        delete_tree(node->left);
        delete_tree(node->right);

        if (!node->left && !node->right) {
            delete (leafnode*)node;
        } else {
            delete node;
        }
    }

private:
    node* m_root;
};

#endif