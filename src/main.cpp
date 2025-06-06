
#include <array>

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

#include "utils.hpp"
#include "glutils.hpp"
#include "viewport.hpp"

class AmenityHandler : public osmium::handler::Handler 
{
    


    // Print info about one amenity to stdout.
    static void print_amenity(const char* type, const char* name, const osmium::geom::Coordinates& c) {
        std::printf("%8.4f,%8.4f %-15s %s\n", c.x, c.y, type, name ? name : "");
    }

    // Calculate the center point of a NodeRefList.
    static osmium::geom::Coordinates calc_center(const osmium::NodeRefList& nr_list) {
        // Coordinates simply store an X and Y coordinate pair as doubles.
        // (Unlike osmium::Location which stores them more efficiently as
        // 32 bit integers.) Use Coordinates when you want to do calculations
        // or store projected coordinates.
        osmium::geom::Coordinates c{ 0.0, 0.0 };

        for (const auto& nr : nr_list) {
            c.x += nr.lon();
            c.y += nr.lat();
        }

        c.x /= static_cast<double>(nr_list.size());
        c.y /= static_cast<double>(nr_list.size());

        return c;
    }

public:

    std::vector<double> node_coords;

    std::vector<double> way_nodecoords;
    std::vector<GLint> way_startidx;
    std::vector<GLsizei> way_counts;

    // The callback functions can be either static or not depending on whether
    // you need to access any member variables of the handler.
    void node(const osmium::Node& node) {
        // Getting a tag value can be expensive, because a list of tags has
        // to be gone through and each tag has to be checked. So we store the
        // result and reuse it.
        const char* amenity = node.tags()["amenity"];
        if (amenity) {
            print_amenity(amenity, node.tags()["name"], node.location());
        }

        auto loc = osmium::geom::MercatorProjection{}(node.location());

        node_coords.push_back(loc.x);
        node_coords.push_back(loc.y);
        node_coords.push_back(0.0);
    }

    void way(const osmium::Way& way)
    {
        auto& nodes = way.nodes();

        way_startidx.push_back(GLint(way_nodecoords.size() / 3));
        way_counts.push_back(GLsizei(nodes.size()));

        for (const auto& nr : nodes) {
            auto loc = osmium::geom::MercatorProjection{}(nr.location());
            way_nodecoords.push_back(loc.x);
            way_nodecoords.push_back(loc.y);
            way_nodecoords.push_back(0.0);
        }
    }

    // The callback functions can be either static or not depending on whether
    // you need to access any member variables of the handler.
    void area(const osmium::Area& area) {
        const char* amenity = area.tags()["amenity"];
        if (amenity) {
            // Use the center of the first outer ring. Because we set
            // create_empty_areas = false in the assembler config, we can
            // be sure there will always be at least one outer ring.
            const auto center = calc_center(*area.cbegin<osmium::OuterRing>());

            print_amenity(amenity, area.tags()["name"], center);
        }
    }

}; // class AmenityHandler


bool osm(AmenityHandler& data_handler)
{
    using index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    using location_handler_t = osmium::handler::NodeLocationsForWays<index_t>;

    try {
        // The input file
        const osmium::io::File input_file{ "assets/maps/testmap.osm" };

        osmium::area::Assembler::config_type assembler_config;
        assembler_config.create_empty_areas = false;

        osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{ assembler_config };

        osmium::relations::read_relations(input_file, mp_manager);

        index_t index;
        location_handler_t location_handler(index);
        

        osmium::io::Reader reader{ input_file, osmium::io::read_meta::no };
        osmium::apply(reader, location_handler, data_handler, 
            mp_manager.handler([&data_handler](const osmium::memory::Buffer& area_buffer) {
                osmium::apply(area_buffer, data_handler);
            }
        ));

        reader.close();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return false;
    }

    return true;
}

void center_coords(const std::vector<double>& coords, std::vector<float>& coords_float)
{
    glm::dvec3 avg_coord(0.0, 0.0, 0.0);
    for (size_t i = 0; i < coords.size(); i += 3) {
        avg_coord.x += coords[i];
        avg_coord.y += coords[i + 1];
        avg_coord.z += coords[i + 2];
    }
    avg_coord /= (coords.size() / 3);

    coords_float.resize(coords.size());

    //center the nodes around the average coordinate
    for (size_t i = 0; i < coords.size(); i += 3) {
        coords_float[i] = coords[i] - avg_coord.x;
        coords_float[i + 1] = coords[i + 1] - avg_coord.y;
        coords_float[i + 2] = coords[i + 2] - avg_coord.z;
    }
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

    window wnd("MAIN_WINDOW", "3D City", 750, 750);
    if (!wnd.ok()) {
        return -1;
    }

    AmenityHandler data_handler;
    osm(data_handler);
    
    // calculate average coordinate in data_handler.node_coords
    if (data_handler.node_coords.empty()) {
        logERROR("No nodes found in OSM data");
        return -1;
    }
    
    std::vector<float> node_coords;
    std::vector<float> way_nodecoords;

    center_coords(data_handler.node_coords, node_coords);
    center_coords(data_handler.way_nodecoords, way_nodecoords);


    // add some sample points for testing
    //AmenityHandler data_handler;
    //data_handler.node_coords.push_back(0.0f);
    //data_handler.node_coords.push_back(0.0f);
    //data_handler.node_coords.push_back(0.0f);
    //data_handler.node_coords.push_back(100.0f);
    //data_handler.node_coords.push_back(100.0f);
    //data_handler.node_coords.push_back(0.0f);
    //data_handler.node_coords.push_back(-100.0f);
    //data_handler.node_coords.push_back(-100.0f);
    //data_handler.node_coords.push_back(0.f);

    shaderstage::info shader_stages[2] = {
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

    //float vertices[] = {
    //    -0.5f, -0.5f, -0.5f,   0.0f, 0.0f,
    //     0.5f, -0.5f, -0.5f,   1.0f, 0.0f,
    //     0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
    //     0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
    //    -0.5f,  0.5f, -0.5f,   0.0f, 1.0f,
    //    -0.5f, -0.5f, -0.5f,   0.0f, 0.0f,
    //                           
    //    -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
    //     0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
    //     0.5f,  0.5f,  0.5f,   1.0f, 1.0f,
    //     0.5f,  0.5f,  0.5f,   1.0f, 1.0f,
    //    -0.5f,  0.5f,  0.5f,   0.0f, 1.0f,
    //    -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
    //                           
    //    -0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //    -0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
    //    -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //    -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //    -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
    //    -0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //                           
    //     0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //     0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
    //     0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //     0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //     0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
    //     0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //                           
    //    -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //     0.5f, -0.5f, -0.5f,   1.0f, 1.0f,
    //     0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
    //     0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
    //    -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
    //    -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
    //                           
    //    -0.5f,  0.5f, -0.5f,   0.0f, 1.0f,
    //     0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
    //     0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //     0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
    //    -0.5f,  0.5f,  0.5f,   0.0f, 0.0f,
    //    -0.5f,  0.5f, -0.5f,   0.0f, 1.0f
    //};
    
    unsigned VAO, VBO;
    unsigned EBO;
    unsigned VAO_way;
    unsigned VBO_way;
    
    glGenVertexArrays(1, &VAO);
    glGenVertexArrays(1, &VAO_way);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &VBO_way);

    //glGenBuffers(1, &EBO);
    
    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, node_coords.size() * sizeof(float), node_coords.data(), GL_STATIC_DRAW);

        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    
        // positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // tex coords
        //glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        //glEnableVertexAttribArray(1);
    }
    glBindVertexArray(0);


    // Create a VAO for the ways
    glBindVertexArray(VAO_way);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_way);
        glBufferData(GL_ARRAY_BUFFER, way_nodecoords.size() * sizeof(float), way_nodecoords.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);




    //int texture0_loc = shader.get_uniform_loc("texture0");
    //int texture1_loc = shader.get_uniform_loc("texture1");
    int mvp_loc = shader.get_uniform_loc("MVP");
    int color_loc = shader.get_uniform_loc("in_color");

    

    //constexpr int num_cubes = 30;
    //std::srand(SDL_GetTicks());
    //auto cubes = std::make_unique<std::pair<glm::vec3, glm::vec3>[]>(num_cubes);
    //for (int i = 0; i < num_cubes; ++i)
    //{
    //    auto pos_xz = glm::circularRand(3.0f);
    //    cubes[i].first = glm::vec3(pos_xz[0], 0.0f, pos_xz[1]);
    //    cubes[i].second = glm::sphericalRand(1.0f);
    //}

    edit_camera camera;

    glm::mat4 projection = glm::ortho(-1000.f, 1000.f, -1000.f, 1000.f, 0.1f, 100.f);//glm::perspective(glm::radians(45.0f), wnd.aspect_ratio(), 0.001f, 100.0f);

    uint64_t last_ticks = 0;

    //SDL_SetRelativeMouseMode(SDL_TRUE);

    bool quit = false;
    while (!quit)
    {
        uint64_t cur_ticks = SDL_GetTicks64();
        uint64_t delta_ticks = cur_ticks - last_ticks;
        last_ticks = cur_ticks;

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

        double time = double(SDL_GetTicks64()) / 1000;
        float sin_curve = float((std::sin(time) + 1.0) / 2.0);

        //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClearColor(1.f, 1.f, 1.f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        shader.use();
        //shader.bind_texture(texture0_loc, 0, containertex);
        //shader.bind_texture(texture1_loc, 1, logotex);

        glBindVertexArray(VAO);
        {
            //for (int i = 0; i < num_cubes; ++i)
            //{
            //    glm::mat4 model(1.0f);
            //    model = glm::translate(model, cubes[i].first);
            //    model = glm::rotate(model, float(time), cubes[i].second);
            //    model = glm::scale(model, glm::vec3(0.5f, 0.5f, 0.5f));
            //
            //    glm::mat4 mvp = projection * camera.view() * model;
            //    glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
            //
            //    glDrawArrays(GL_TRIANGLES, 0, 36);
            //}
            //
            //glm::mat4 model(1.0f);
            //model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));
            //
            //glm::mat4 mvp = projection * camera.view() * model;
            //glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
            //glDrawArrays(GL_TRIANGLES, 0, 36);

            glm::mat4 model(1.0f);
            glm::mat4 mvp = projection * camera.view() * model;
            glUniform4f(color_loc, 0.0f, 0.0f, 0.0f, 1.0f);
            glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
            glDrawArrays(GL_POINTS, 0, node_coords.size() / 3);
        }
        glBindVertexArray(0);

        // Draw the ways
        glBindVertexArray(VAO_way);
        {
            glm::mat4 model(1.0f);
            glm::mat4 mvp = projection * camera.view() * model;
            glUniform4f(color_loc, 1.0f, 0.0f, 0.0f, 1.0f);
            glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));

            glMultiDrawArrays(GL_LINE_STRIP, 
                data_handler.way_startidx.data(), 
                data_handler.way_counts.data(),
                GLsizei(data_handler.way_counts.size()));
        }
        glBindVertexArray(0);


        wnd.update();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    //glDeleteBuffers(1, &EBO);

    return 0;
}