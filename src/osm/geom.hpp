
#ifndef OSM_GEOM_HPP
#define OSM_GEOM_HPP

#include <utility>
#include <vector>
#include <limits>
#include <span>

#include <glm/glm.hpp>
#include <osmium/geom/coordinates.hpp>

#include "../utils.hpp"


using osmpoint = osmium::geom::Coordinates;
using osmsegment = std::pair<osmpoint, osmpoint>;

// Polygon orientation
enum orient
{
    ORIENT_CCW, // counter-clockwise
    ORIENT_CW,  // clockwise
    ORIENT_COLL // collinear
};

// Check for proper intersection of line segments (i.e. not parallel, and not at endpoints).
bool segments_proper_intersect(const osmsegment& seg1, const osmsegment& seg2);

// Check if a polygon is within the bounds of another polygon (or on the border).
bool polygon_covered_by(std::span<const osmpoint> inner_poly, std::span<const osmpoint> outer_poly);

// Get orientation of polygon.
orient polygon_orient(std::span<const osmpoint> poly);

// Triangulate polygon.
// Input and output are clockwise oriented.
// \param reverse_orient Reverse orientation of input vertices.
std::vector<uint32_t> polygon_triangulate(std::span<const osmpoint> poly, bool reverse_orient = false);


// Axis-aligned bounding box.
struct bbox2d
{
    glm::dvec2 min;
    glm::dvec2 max;

    bbox2d() :
        min(std::numeric_limits<double>::infinity()),
        max(-std::numeric_limits<double>::infinity())
    {}

    void extend(const bbox2d& other)
    {
        this->min = glm::min(this->min, other.min);
        this->max = glm::max(this->max, other.max);
    }

    void extend(glm::dvec2 point)
    {
        this->min = glm::min(this->min, point);
        this->max = glm::max(this->max, point);
    }

    glm::dvec2 center() const {
        return (min + max) / 2.0;
    }

    bool intersects(const bbox2d& rhs) const
    {
        // Check if there is some overlap on the right 
        // (i.e. min before other box's max) and some overlap 
        // on the left (max after other box's min)
        return
            this->min.x <= rhs.max.x &&
            this->min.y <= rhs.max.y &&
            this->max.x >= rhs.min.x &&
            this->max.y >= rhs.min.y;
    }
};

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