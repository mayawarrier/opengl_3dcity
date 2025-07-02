
#ifndef AABB_TREE_HPP
#define AABB_TREE_HPP

#include "../utils.hpp"
#include "geom.hpp"

template <typename T>
struct aabb_traits
{
    bbox2d b;
    static const bbox2d& get_bbox(const T& obj) {
        (void)obj;
        return b;
    }
};

// Axis-aligned bounding box tree.
// Accelerates intersection queries.
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

    struct leafnode : public node
    {
        T data;
    };

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
        std::vector<node*> candidates;

        auto insert_if_intersects = [&](node* node) {
            if (node && node->bbox.intersects(bbox))
                candidates.push_back(node);
        };

        insert_if_intersects(m_root);

        while (!candidates.empty())
        {
            node* node = candidates.back();
            candidates.pop_back();

            // If it intersects, descend further down the tree
            insert_if_intersects(node->left);
            insert_if_intersects(node->right);

            if (!node->left && !node->right) {
                auto* leaf = (leafnode*)node;
                ret.push_back(leaf->data);
            }
        }
        return ret;
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
            node->bbox = aabb_traits<T>::get_bbox(objects[0]);
            node->data = objects[0];
            return node;
        }
        else {
            auto* n = new node();

            for (size_t i = 0; i < num_objects; ++i) {
                n->bbox.extend(aabb_traits<T>::get_bbox(objects[i]));
            }

            glm::vec2 dim_sizes = n->bbox.max - n->bbox.min;
            int longest_dim = dim_sizes.x > dim_sizes.y ? 0 : 1;

            std::sort(objects, objects + num_objects,
                [&longest_dim](const T& lhs, const T& rhs) 
                {
                    double lhs_dim = aabb_traits<T>::get_bbox(lhs).center()[longest_dim];
                    double rhs_dim = aabb_traits<T>::get_bbox(rhs).center()[longest_dim];
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