
#include <array>
#include <charconv>

#include <SDL2/SDL.h>
#include <SDL2/SDL_main.h>

#include "glad/glad.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "utils.hpp"
#include "glutils.hpp"
#include "viewport.hpp"
#include "osm/osm.hpp"



int main(int argc, char* argv[]) 
{
    if (!log_init("3dcity.log")) {
        return -1;
    }

#ifdef _WIN32
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, "permonitorv2");
#endif

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        logERROR("SDL_Init(): %s", SDL_GetError());
        return false;
    }
    if (std::atexit(SDL_Quit) != 0) {
        logERROR("atexit(SDL_Quit) failed");
        SDL_Quit();
        return -1;
    }

    window wnd("MAIN_WINDOW", "3D City", { 900, 750 });
    if (!wnd.ok()) {
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PRIMITIVE_RESTART);
    glPrimitiveRestartIndex(std::numeric_limits<uint32_t>::max());

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

        
        // Draw lines
        glBindVertexArray(VAO_lines);
        {
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