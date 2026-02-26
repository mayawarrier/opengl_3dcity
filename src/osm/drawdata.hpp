
#ifndef OSM_DRAWDATA_HPP
#define OSM_DRAWDATA_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <glm/glm.hpp>

// Raw data sent to OpenGL for drawing.
template <typename TVert>
struct draw_data
{
    std::string name;
    glm::vec4 color;
    std::vector<TVert> verts;
    std::vector<uint32_t> tri_indices; // GL_TRIANGLES

    uint32_t add_vertex(TVert x, TVert y, TVert z)
    {
        uint32_t idx = num_verts();
        verts.push_back(x);
        verts.push_back(y);
        verts.push_back(z);
        return idx;
    }

    uint32_t add_vertex(const glm::tvec3<TVert>& vert) {
        return add_vertex(vert.x, vert.y, vert.z);
    }

    void add_triangle(uint32_t idx0, uint32_t idx1, uint32_t idx2) {
        tri_indices.push_back(idx0);
        tri_indices.push_back(idx1);
        tri_indices.push_back(idx2);
    }

    void add_triangle_w_offset(uint32_t idx0, uint32_t idx1, uint32_t idx2, uint32_t offset) {
        add_triangle(idx0 + offset, idx1 + offset, idx2 + offset);
    }

    glm::dvec3 get_vertex(uint32_t idx) {
        return { verts[3 * idx], verts[3 * idx + 1], verts[3 * idx + 2] };
    }

    uint32_t num_verts() const { return uint32_t(verts.size() / 3); }
    uint32_t num_tris() const { return uint32_t(tri_indices.size() / 3); }
};

using draw_dataf = draw_data<float>;
using draw_datad = draw_data<double>;

#endif