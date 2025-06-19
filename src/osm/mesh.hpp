
#ifndef AABB_TREE_HPP
#define AABB_TREE_HPP

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include <osmium/geom/coordinates.hpp>
#include <osmium/fwd.hpp>


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

    glm::vec2 center() const {
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

enum orient
{
    ORIENT_CCW, // counter-clockwise
    ORIENT_CW,  // clockwise
    ORIENT_COLL // collinear
};

template <typename TVert>
struct draw_data
{
    std::string name;
    std::vector<TVert> verts;
    std::vector<uint32_t> tri_indices;
    std::vector<uint32_t> line_indices;
};

using draw_dataf = draw_data<float>;
using draw_datad = draw_data<double>;


// Matches buildings to their parts and converts them into
// a set of meshes/lines that can be rendered by opengl.
class building_assembler
{
public:
    bool add_building(const osmium::Way& way, 
        const char* name, bool is_part, double ht_bottom, double ht_top);

    std::vector<draw_datad> get_draw_data();

public:
    struct part
    {
        osmium::object_id_type id;
        orient orient;
        bbox2d bbox;
        double ht_btm, ht_top;
        std::vector<osmium::geom::Coordinates> coords;
    };

    struct building
    {
        std::string name;
        part info;
        std::vector<part*> parts;
    };

private:
    bool get_part(const osmium::Way& way,
        double ht_bottom, double ht_top, part& part);

    std::vector<building> m_buildings;
    std::vector<part> m_parts;
};

#endif
