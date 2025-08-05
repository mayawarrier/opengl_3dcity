
#ifndef AABB_TREE_HPP
#define AABB_TREE_HPP

#include <functional>

#include "../common.hpp"
#include "../geom.hpp"

template <typename T>
struct aabb_tree_traits
{
    static_assert(deferred_false<T>::value,
        "aabb_tree_traits must be specialized for the type T.");

    bbox2d bb;
    static const bbox2d& bbox(const T& obj) { (void)obj; return bb; }
};

// Axis-aligned bounding box tree.
// T = type of objects stored in the tree. 
// Assumed to be cheap to copy or a pointer type.
// The tree does not own the objects!
template <typename T>
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

    using traits = aabb_tree_traits<T>;

public:
    aabb_tree() :
        m_root(nullptr)
    {}

    // Changes the order of the source array!
    static aabb_tree create_unsafe(T* objects, size_t num_objects) {
        return { objects, num_objects };
    }

    MOVE_ONLY_CLASS(aabb_tree, m_root, nullptr)

    std::vector<T> intersect(const bbox2d& bbox) const
    {
        std::vector<T> ret;

        std::vector<node*> nodes;
        auto insert_if_intersects = [&](node* node) {
            if (node && bbox_intersects_bbox(node->bbox, bbox))
                nodes.push_back(node);
        };

        insert_if_intersects(m_root);

        while (!nodes.empty())
        {
            node* node = nodes.back();
            nodes.pop_back();

            // If it intersects, search further down the tree
            insert_if_intersects(node->left);
            insert_if_intersects(node->right);

            if (!node->left && !node->right) {
                ret.push_back(((leafnode*)node)->data);
            }
        }
        return ret;
    }

    // \param intersects_object
    // callable of type bool(const ray2d&, const T&, double& out_dist, const param_range&)
    bool ray_first_intersect(const ray2d& ray, auto ray_intersects_object,
        double& out_t, T& out_object, const param_range& t_range = {}) const
    {
        double min_hit_t = std::numeric_limits<double>::infinity();
        T* min_hit_object = nullptr;

        std::vector<node*> nodes;
        auto insert_if_intersects = [&](node* node) 
        {
            if (node) {
                double t = -std::numeric_limits<double>::infinity();
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

        insert_if_intersects(m_root);
        
        while (!nodes.empty())
        {
            node* node = nodes.back();
            nodes.pop_back();

            // If it intersects, search further down the tree
            insert_if_intersects(node->left);
            insert_if_intersects(node->right);

            if (!node->left && !node->right) 
            {
                auto* leaf = (leafnode*)node;

                double t;
                if (std::invoke(ray_intersects_object, ray, leaf->data, t, t_range) && t < min_hit_t) {
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
    aabb_tree(T* objects, size_t num_objects) :
        m_root(make_tree(objects, num_objects))
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
            node->bbox = traits::bbox(objects[0]);
            node->data = objects[0];
            return node;
        }
        else {
            auto* n = new node();

            for (size_t i = 0; i < num_objects; ++i) {
                n->bbox.extend(traits::bbox(objects[i]));
            }

            glm::vec2 dim_sizes = n->bbox.max - n->bbox.min;
            int longest_dim = dim_sizes.x > dim_sizes.y ? 0 : 1;

            std::sort(objects, objects + num_objects,
                [&longest_dim](const T& lhs, const T& rhs) 
                {
                    double lhs_dim = traits::bbox(lhs).center()[longest_dim];
                    double rhs_dim = traits::bbox(rhs).center()[longest_dim];
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