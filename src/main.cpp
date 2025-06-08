
#include <array>
#include <charconv>

#include <SDL2/SDL.h>
#include <SDL2/SDL_main.h>

#include "glad/glad.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <mapbox/earcut.hpp>

#include "utils.hpp"
#include "glutils.hpp"
#include "viewport.hpp"


namespace mapbox {
namespace util {

template <>
struct nth<0, osmium::geom::Coordinates> {
    inline static auto get(const osmium::geom::Coordinates& t) {
        return t.x;
    };
};
template <>
struct nth<1, osmium::geom::Coordinates> {
    inline static auto get(const osmium::geom::Coordinates& t) {
        return t.y;
    };
};
}}

// strcmp with null check
static bool str_equal(const char* str1, const char* str2) 
{
    if (str1 && !str2) { return false; }
    if (str2 && !str1) { return false; }
    if (!str2 && !str1) { return true; }
    return std::strcmp(str1, str2) == 0;
}

static osmium::geom::Coordinates calc_center(const osmium::NodeRefList& nr_list)
{
    osmium::geom::Coordinates c{ 0.0, 0.0 };
    for (const auto& nr : nr_list) {
        c.x += nr.lon();
        c.y += nr.lat();
    }

    c.x /= static_cast<double>(nr_list.size());
    c.y /= static_cast<double>(nr_list.size());
    return c;
}

// Every 3 values is a vertex.
template <typename T>
static glm::vec<3, T, glm::defaultp> avg_vert(const std::vector<T>& verts)
{
    glm::vec<3, T, glm::defaultp> avg(T(0));
    for (size_t i = 0; i < verts.size(); i += 3) {
        avg.x += verts[i];
        avg.y += verts[i + 1];
        avg.z += verts[i + 2];
    }
    avg /= (verts.size() / 3);
    return avg;
}

struct osm_data
{
    std::vector<float> verts;
    
    // GL_TRIANGLES
    std::vector<uint32_t> tri_indices;
    // GL_LINE_STRIP with prim restart index
    std::vector<uint32_t> line_indices;
};

class osm_datahandler : public osmium::handler::Handler 
{
public:
    void node(const osmium::Node& node) 
    {
        //auto loc = osmium::geom::MercatorProjection{}(node.location());
        //node_coords.push_back(loc.x);
        //node_coords.push_back(loc.y);
        //node_coords.push_back(0.0);
    }

    void way(const osmium::Way& way)
    {
        auto& tags = way.tags();
        auto& nodes = way.nodes();

        if (tags.has_key("highway"))
        {
            add_polyline_from_nodes(nodes);
        }
        else if (tags.has_key("building") || str_equal(tags["building:part"], "yes"))
        {
            const char* height_str, *levels_str;
            if ((height_str = tags["height"]))
            {
                double height;
                std::from_chars(height_str, height_str + std::strlen(height_str), height);
                add_building(nodes, height);
            }
            else if ((levels_str = tags["building:levels"]))
            {
                int levels;
                std::from_chars(levels_str, levels_str + std::strlen(levels_str), levels);
                add_building(nodes, levels * 3.0);
            }
            else { add_building(nodes, 3.0); }
        }
    }

    // The callback functions can be either static or not depending on whether
    // you need to access any member variables of the handler.
    void area(const osmium::Area& area) 
    {
        const char* name = area.tags()["name"];
        //logMESSAGE("Received area %s", name);

        //const char* amenity = area.tags()["amenity"];
        //if (amenity) {
        //    // Use the center of the first outer ring. Because we set
        //    // create_empty_areas = false in the assembler config, we can
        //    // be sure there will always be at least one outer ring.
        //    const auto center = calc_center(*area.cbegin<osmium::OuterRing>());
        //
        //    print_amenity(amenity, area.tags()["name"], center);
        //}
    }

    osm_data to_osmdata()
    {
        auto center = avg_vert(m_verts);

        std::vector<float> verts_float;
        verts_float.resize(m_verts.size());

        for (size_t i = 0; i < m_verts.size(); i += 3) {
            verts_float[i + 0] = m_verts[i + 0] - center.x;
            verts_float[i + 1] = m_verts[i + 1] - center.y;
            verts_float[i + 2] = m_verts[i + 2] - center.z;
        }
        return {
            .verts = std::move(verts_float),
            .tri_indices = std::move(m_tri_indices),
            .line_indices = std::move(m_line_indices)
        };
    }

private:
    void add_line_indices(uint32_t idx0, uint32_t idx1)
    {
        m_line_indices.push_back(idx0);
        m_line_indices.push_back(idx1);
        // prim restart index
        m_line_indices.push_back(std::numeric_limits<uint32_t>::max());
    }

    void add_polyline_indices(uint32_t startidx, uint32_t size, bool is_closed)
    {
        if (size == 0) { return; } // handle underflow

        for (uint32_t i = 0; i < size - 1; ++i) {
            m_line_indices.push_back(startidx + i);
        }
        if (is_closed) {
            m_line_indices.push_back(startidx);
        } else {
            m_line_indices.push_back(startidx + size - 1);
        }
        // prim restart index
        m_line_indices.push_back(std::numeric_limits<uint32_t>::max());
    }

    void add_polyline_from_nodes(const osmium::NodeRefList& nodes)
    {
        uint32_t vert_startidx = uint32_t(m_verts.size() / 3);

        for (const auto& node : nodes) {
            auto loc = osmium::geom::MercatorProjection{}(node.location());
            m_verts.push_back(loc.x);
            m_verts.push_back(loc.y);
            m_verts.push_back(0.0);
        }
        add_polyline_indices(vert_startidx, uint32_t(nodes.size()), nodes.is_closed());
    }

    void add_polyline(const std::vector<osmium::geom::Coordinates>& verts, double height, bool is_closed)
    {
        uint32_t vert_startidx = uint32_t(m_verts.size() / 3);

        for (const auto& vert : verts) {
            m_verts.push_back(vert.x);
            m_verts.push_back(vert.y);
            m_verts.push_back(height);
        }
        add_polyline_indices(vert_startidx, uint32_t(verts.size()), is_closed);
    }

    // Verts must be in clockwise winding order
    void add_tri_indices(uint32_t idx0, uint32_t idx1, uint32_t idx2)
    {
        m_tri_indices.push_back(idx0);
        m_tri_indices.push_back(idx1);
        m_tri_indices.push_back(idx2);
    }

    uint32_t add_polygon(const std::vector<osmium::geom::Coordinates>& verts,
        const std::vector<uint32_t>& indices, double height)
    {
        uint32_t vert_startidx = uint32_t(m_verts.size() / 3);

        for (const auto& vert : verts) {
            m_verts.push_back(vert.x);
            m_verts.push_back(vert.y);
            m_verts.push_back(height);
        }
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            add_tri_indices(
                indices[i] + vert_startidx, 
                indices[i + 1] + vert_startidx, 
                indices[i + 2] + vert_startidx);
        }
        return vert_startidx;
    }

    void add_building(const osmium::NodeRefList& nodes, double height)
    {
        std::vector<std::vector<osmium::geom::Coordinates>> earcut_polylines;
        earcut_polylines.resize(1); // earcut needs a vector of polylines

        auto& node_verts = earcut_polylines[0];
        for (const auto& node : nodes) {
            auto loc = osmium::geom::MercatorProjection{}(node.location());
            node_verts.push_back(loc);
        }

        auto topbot_indices = mapbox::earcut<uint32_t>(earcut_polylines);

        uint32_t bot_verts_idx = add_polygon(node_verts, topbot_indices, 0.0);    // bottom face
        uint32_t top_verts_idx = add_polygon(node_verts, topbot_indices, height); // top face

        // bottom and top outlines
        add_polyline(node_verts, 0.0, nodes.is_closed());
        add_polyline(node_verts, height, nodes.is_closed());

        // sides
        for (uint32_t cur = 0; cur < nodes.size(); ++cur)
        {
            uint32_t next = (cur + 1) % nodes.size();

            uint32_t quad[4] = {
                bot_verts_idx + cur,
                bot_verts_idx + next,
                top_verts_idx + cur,
                top_verts_idx + next,
            };
            // faces
            add_tri_indices(quad[0], quad[3], quad[2]);
            add_tri_indices(quad[0], quad[1], quad[3]);

            // outlines
            add_line_indices(quad[0], quad[2]);
            add_line_indices(quad[1], quad[3]);
        }
    }

private:
    //std::vector<double> node_coords;

    std::vector<double> m_verts;
    std::vector<uint32_t> m_line_indices;
    std::vector<uint32_t> m_tri_indices;
};


static bool read_osmfile(const fs::path& path, osm_data& out_data)
{
    using index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    using location_handler_t = osmium::handler::NodeLocationsForWays<index_t>;

    try {
        const osmium::io::File input_file{ path.string() };

        osmium::area::Assembler::config_type assembler_config;
        assembler_config.create_empty_areas = false;

        osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{ assembler_config };
        osmium::relations::read_relations(input_file, mp_manager);

        index_t index;
        location_handler_t location_handler(index);
        osm_datahandler data_handler;
        
        osmium::io::Reader reader{ input_file, osmium::io::read_meta::no };
        osmium::apply(reader, location_handler, data_handler,
            mp_manager.handler([&data_handler](const osmium::memory::Buffer& area_buffer) {
                osmium::apply(area_buffer, data_handler);
            }
        ));
        reader.close();

        out_data = data_handler.to_osmdata();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return false;
    }

    return true;
}


int main(int argc, char* argv[]) 
{
    if (!log_init("3dcity.log")) {
        return -1;
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        logERROR("SDL_Init(): %s", SDL_GetError());
        return false;
    }
    if (std::atexit(SDL_Quit) != 0) {
        logERROR("atexit(SDL_Quit) failed");
        SDL_Quit();
        return -1;
    }

    window wnd("MAIN_WINDOW", "3D City", 900, 750);
    if (!wnd.ok()) {
        return -1;
    }

    //wnd.set_fullscreen(true);

    osm_data osmdata;
    if (!read_osmfile("assets/maps/testmap.osm", osmdata)) {
        logERROR("Failed to read OSM map file");
        return -1;
    }

    // add some sample points for testing
    //osm_datahandler osmdata;
    //osmdata.node_coords.push_back(0.0f);
    //osmdata.node_coords.push_back(0.0f);
    //osmdata.node_coords.push_back(0.0f);
    //osmdata.node_coords.push_back(100.0f);
    //osmdata.node_coords.push_back(100.0f);
    //osmdata.node_coords.push_back(0.0f);
    //osmdata.node_coords.push_back(-100.0f);
    //osmdata.node_coords.push_back(-100.0f);
    //osmdata.node_coords.push_back(0.f);

    shaderfile::info shader_stages[2] = {
        { "assets/shaders/vertex.vert", GL_VERTEX_SHADER },
        { "assets/shaders/fragment.frag", GL_FRAGMENT_SHADER }
    };
    shader shader(shader_stages, 2);
    if (!shader.ok()) {
        return -1;
    }

    //texture2d containertex("assets/textures/container.jpg");
    //if (!containertex.ok()) {
    //    return -1;
    //}
    //
    //texture2d logotex("assets/textures/Opengl-logo.png");
    //if (!logotex.ok()) {
    //    return -1;
    //}

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PRIMITIVE_RESTART);

    glPrimitiveRestartIndex(std::numeric_limits<uint32_t>::max());
    
    unsigned VBO_verts;

    unsigned EBO_lines;
    unsigned VAO_lines;
    //unsigned VBO_way;

    unsigned EBO_tris;
    unsigned VAO_tris;
    
    //glGenVertexArrays(1, &VAO);
    //glGenBuffers(1, &VBO);

    glGenBuffers(1, &VBO_verts);

    glGenVertexArrays(1, &VAO_lines);
    glGenBuffers(1, &EBO_lines);

    glGenVertexArrays(1, &VAO_tris);
    glGenBuffers(1, &EBO_tris);


    //glBindBuffer(GL_ARRAY_BUFFER, VBO_verts);
    //glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);

    //glGenBuffers(1, &EBO);
    
    //glBindVertexArray(VAO);
    //{
    //    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //    glBufferData(GL_ARRAY_BUFFER, node_coords.size() * sizeof(float), node_coords.data(), GL_STATIC_DRAW);
    //
    //    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    //    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    //
    //    // positions
    //    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    //    glEnableVertexAttribArray(0);
    //
    //    // tex coords
    //    //glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    //    //glEnableVertexAttribArray(1);
    //}
    //glBindVertexArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_verts);
    glBufferData(GL_ARRAY_BUFFER, osmdata.verts.size() * sizeof(float), osmdata.verts.data(), GL_STATIC_DRAW);

    glBindVertexArray(VAO_lines);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_verts);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_lines);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, osmdata.line_indices.size() * sizeof(uint32_t), osmdata.line_indices.data(), GL_STATIC_DRAW);
    
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);


    glBindVertexArray(VAO_tris);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_verts);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_tris);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, osmdata.tri_indices.size() * sizeof(uint32_t), osmdata.tri_indices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);


    //int texture0_loc = shader.get_uniform_loc("texture0");
    //int texture1_loc = shader.get_uniform_loc("texture1");
    int mvp_loc = shader.get_uniform_loc("MVP");
    int color_loc = shader.get_uniform_loc("in_color");


    edit_camera camera;

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), wnd.aspect_ratio(), 0.1f, 10000.0f);
    //glm::mat4 projection = glm::ortho(-1000.f, 1000.f, -1000.f, 1000.f, 0.001f, 1000.f);

    uint64_t last_ticks = 0;

    //SDL_SetRelativeMouseMode(SDL_TRUE);

    bool quit = false;
    while (!quit)
    {
        uint64_t cur_ticks = SDL_GetTicks64();
        uint64_t delta_ticks = cur_ticks - last_ticks;
        last_ticks = cur_ticks;

        double time = double(SDL_GetTicks64()) / 1000;

        camera.new_frame();

        // todo: for multiple windows, events should be sent based on window ID
        // Each window would receive events via process_event
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                quit = true;
                break;

            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    quit = true;
                }
                break;

            default: break;
            }

            camera.process_event(event);
        }

        camera.update();

        glClearColor(1.f, 1.f, 1.f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        shader.use();
        //shader.bind_texture(texture0_loc, 0, containertex);
        //shader.bind_texture(texture1_loc, 1, logotex);

        glm::mat4 model(1.0f);
        model = glm::rotate(model, glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));
        glm::mat4 mvp = projection * camera.view() * model;
        glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));

        //glBindVertexArray(VAO);
        //{
        //    glUniform4f(color_loc, 0.0f, 0.0f, 0.0f, 1.0f); // black
        //    glDrawArrays(GL_POINTS, 0, node_coords.size() / 3);
        //}
        //glBindVertexArray(0);

        
        glBindVertexArray(VAO_lines);
        {
            // Draw lines
            glUniform4f(color_loc, 0.0f, 0.0f, 0.0f, 1.0f); // black
            glDrawElements(GL_LINE_STRIP, osmdata.line_indices.size(), GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);

        // Draw meshes
        glBindVertexArray(VAO_tris);
        {
            glUniform4f(color_loc, 0.5f, 0.5f, 0.5f, 1.0f); // gray
            glDrawElements(GL_TRIANGLES, osmdata.tri_indices.size(), GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);

        wnd.update();
    }

    //glDeleteVertexArrays(1, &VAO);
    //glDeleteBuffers(1, &VBO);
    //glDeleteBuffers(1, &EBO);

    return 0;
}