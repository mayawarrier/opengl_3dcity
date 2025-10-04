
#ifndef OSM_AABB_TREE_HPP
#define OSM_AABB_TREE_HPP

#include <functional>
#include <boost/container/small_vector.hpp>

#include "../common.hpp"
#include "../geom.hpp"
#include "fwd.hpp"


// Axis-aligned bounding box tree.
// Does not own the objects stored. It is expected that the object type 
// is a pointer or a cheap-to-copy type that points to the actual data.
template <typename T, typename Traits>
class aabb_tree
{
private:
    struct node 
    {
        node* left;
        node* right;
        bbox2d bbox;
    };
    struct leafnode : public node {
        T data;
    };

    static_assert(requires {
        { Traits::bbox(std::declval<T>()) } -> std::same_as<const bbox2d&>;
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

    std::vector<T> intersect(const bbox2d& bbox) const
    {
        std::vector<T> ret;

        types::small_vector<node*, 16> nodes;
        auto push_node_if_intersects = [&](node* node) {
            if (node && bbox_intersects_bbox(node->bbox, bbox))
                nodes.push_back(node);
        };

        push_node_if_intersects(m_root);

        while (!nodes.empty())
        {
            node* node = nodes.back();
            nodes.pop_back();

            // If it intersects, search further down the tree
            push_node_if_intersects(node->left);
            push_node_if_intersects(node->right);

            if (!node->left && !node->right) {
                ret.push_back(((leafnode*)node)->data);
            }
        }
        return ret;
    }

    // \param ray_hits_object_cb -
    // callback of type bool(const ray2d&, const T&, double& out_t, param_range, double eps)
    bool ray_first_hit(const ray2d& ray, auto ray_hits_object_cb, 
        double& out_t, T& out_object, param_range t_range = {}, double eps = 1e-9) const
    {
        double min_hit_t = std::numeric_limits<double>::infinity();
        T* min_hit_object = nullptr;

        // sufficient for trees with upto 2^16 objects
        types::small_vector<node*, 16> nodes;
        auto push_node_if_hit = [&](node* node) 
        {
            if (node) {
                double t = std::numeric_limits<double>::infinity();
                auto inter_type = ray_intersects_bbox(ray, node->bbox, t, t_range);

                // No intersection or too far, do not search this subtree.
                // RAYBOX_INTER_INSIDE means ray might still 
                // hit the object even if t is larger than min_hit_t
                bool skip_node = inter_type == RAYBOX_INTER_NONE ||
                    (inter_type == RAYBOX_INTER_BORDER && t >= min_hit_t);

                if (!skip_node) {
                    nodes.push_back(node);
                }
            }
        };

        push_node_if_hit(m_root);
        
        while (!nodes.empty())
        {
            node* node = nodes.back();
            nodes.pop_back();

            // If it intersects, search further down the tree
            push_node_if_hit(node->left);
            push_node_if_hit(node->right);

            if (!node->left && !node->right) 
            {
                auto* leaf = (leafnode*)node;

                double t = std::numeric_limits<double>::infinity();
                bool inter = std::invoke(ray_hits_object_cb, ray, leaf->data, t, t_range, eps);
                if (inter && t < min_hit_t) {
                    min_hit_t = t;
                    min_hit_object = &leaf->data;
                }
            }
        }

        if (min_hit_object) {
            out_t = min_hit_t;
            out_object = *min_hit_object;
            return true;
        }
        else { return false; }
    }

    ~aabb_tree() {
        delete_tree(m_root);
    }

private:
    aabb_tree(std::span<T> objects) :
        m_root(make_tree(objects.data(), objects.size()))
    {}

    static node* make_tree(T* objects, size_t num_objects)
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

            glm::vec2 dim_sizes = n->bbox.max - n->bbox.min;
            int longest_dim = dim_sizes.x > dim_sizes.y ? 0 : 1;

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

    static void delete_tree(node* node)
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