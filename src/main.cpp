
// todo: this code is a giant mess!

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
#include "osm/common.hpp"



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

    osm_gl_draw_data osmdata;
    if (!read_osmfile("assets/maps/testmap_larger.osm", osmdata)) {
        logERROR("Failed to read OSM map file");
        return -1;
    }

    window wnd("MAIN_WINDOW", "3D City", { 1100, 900 });
    if (!wnd.ok()) {
        return -1;
    }

    //glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_PRIMITIVE_RESTART);
    //glPrimitiveRestartIndex(std::numeric_limits<uint32_t>::max());

    bool DRAW_WIREFRAME = false;
    bool DRAW_TRIANGLES = true;

    shaderfile::info main_shaderfiles[2] = {
        { "assets/shaders/vertex.glsl", GL_VERTEX_SHADER },
        { "assets/shaders/fragment.glsl", GL_FRAGMENT_SHADER }
    };
    shader shader(main_shaderfiles, 2);
    if (!shader.ok()) {
        return -1;
    }

    shaderfile::info light_shaderfiles[2] = {
        { "assets/shaders/light_vertex.glsl", GL_VERTEX_SHADER },
        { "assets/shaders/light_fragment.glsl", GL_FRAGMENT_SHADER }
    };
    ::shader light_shader(light_shaderfiles, 2);
    if (!light_shader.ok()) {
        return -1;
    }

    unsigned VBO_verts;
    unsigned EBO_tris[NUM_TRI_TYPES];
    unsigned VAO_tris;

    glGenBuffers(1, &VBO_verts);
    glGenBuffers(NUM_TRI_TYPES, EBO_tris);
    glGenVertexArrays(1, &VAO_tris);

    glBindVertexArray(VAO_tris);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_verts);
        glBufferData(GL_ARRAY_BUFFER, osmdata.verts.size() * sizeof(osm_gl_draw_data::vertex), osmdata.verts.data(), GL_STATIC_DRAW);

        for (int i = 0; i < NUM_TRI_TYPES; ++i)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_tris[i]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, osmdata.tris[i].size() * sizeof(osm_gl_draw_data::tri), osmdata.tris[i].data(), GL_STATIC_DRAW);
        }

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    }
    glBindVertexArray(0);

    unsigned VBO_light;
    unsigned VAO_light;

    glGenBuffers(1, &VBO_light);
    glGenVertexArrays(1, &VAO_light);

    float light_verts[] = {
        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,

        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,

         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,

        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f,
    };

    glBindVertexArray(VAO_light);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_light);
        glBufferData(GL_ARRAY_BUFFER, sizeof(light_verts), light_verts, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);


    //int texture0_loc = shader.get_uniform_loc("texture0");
    //int texture1_loc = shader.get_uniform_loc("texture1");
    int Tmodel_loc = shader.get_uniform_loc("Tmodel");
    int Tview_loc = shader.get_uniform_loc("Tview");
    int Tproj_loc = shader.get_uniform_loc("Tproj");
    int Tnormal_loc = shader.get_uniform_loc("Tnormal");
    int light_pos_loc = shader.get_uniform_loc("light_pos");
    int light_color_loc = shader.get_uniform_loc("light_color");
    int obj_color_loc = shader.get_uniform_loc("object_color");

    shader.use();

    glm::mat4 Tproj = glm::perspective(glm::radians(45.0f), wnd.aspect_ratio(), 0.1f, 50000.0f);
    //glm::mat4 projection = glm::ortho(-1000.f, 1000.f, -1000.f, 1000.f, 0.001f, 1000.f);
    glUniformMatrix4fv(Tproj_loc, 1, GL_FALSE, glm::value_ptr(Tproj));

    glm::mat4 Tmodel(1.0f);
    Tmodel = glm::rotate(Tmodel, glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));
    glUniformMatrix4fv(Tmodel_loc, 1, GL_FALSE, glm::value_ptr(Tmodel));

    // glm::transpose(glm::inverse(glm::mat3(Tmodel))); // not needed since Tmodel doesn't do non-uniform scaling
    glm::mat3 Tnormal = Tmodel;//glm::transpose(glm::inverse(glm::mat3(Tmodel)));
    glUniformMatrix3fv(Tnormal_loc, 1, GL_FALSE, glm::value_ptr(Tnormal));

    bbox2d bb; float maxZ = 0.f;
    for (const auto& vert : osmdata.verts) {
        bb.extend(glm::dvec2(vert.pos[0], vert.pos[1]));
        if (vert.pos[2] > maxZ) {
            maxZ = vert.pos[2];
        }
    }
    auto center = bb.center();
    auto light_pos = glm::fvec3(float(center.x), float(center.y), 1.5 * maxZ);
    light_pos = glm::fvec3(Tmodel * glm::fvec4(light_pos, 1.f));

    glUniform3f(light_pos_loc, light_pos.x, light_pos.y, light_pos.z);
    glUniform3f(light_color_loc, 1.0f, 1.f, 1.f);


    int light_shader_Tmodel_loc = light_shader.get_uniform_loc("Tmodel");
    int light_shader_Tview_loc = light_shader.get_uniform_loc("Tview");
    int light_shader_Tproj_loc = light_shader.get_uniform_loc("Tproj");
    int light_shader_light_color_loc = light_shader.get_uniform_loc("light_color");

    light_shader.use();

    glm::mat4 light_Tmodel(1.0f);
    light_Tmodel = glm::translate(light_Tmodel, light_pos);
    light_Tmodel = glm::scale(light_Tmodel, glm::vec3(0.1f * maxZ)); // scale light cube based on map size

    glUniformMatrix4fv(light_shader_Tmodel_loc, 1, GL_FALSE, glm::value_ptr(light_Tmodel));
    glUniformMatrix4fv(light_shader_Tproj_loc, 1, GL_FALSE, glm::value_ptr(Tproj));    
    glUniform3f(light_shader_light_color_loc, 1.0f, 0.f, 0.f);

    edit_camera camera;
    uint64_t last_ticks = 0;
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

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        shader.use();
        //shader.bind_texture(texture0_loc, 0, containertex);
        //shader.bind_texture(texture1_loc, 1, logotex);

        glUniformMatrix4fv(Tview_loc, 1, GL_FALSE, glm::value_ptr(camera.view()));
        
        // Draw meshes
        glBindVertexArray(VAO_tris);
        {
            for (int i = 0; i < NUM_TRI_TYPES; ++i)
            {
                if (osmdata.tris[i].empty()) {
                    continue;
                }

                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_tris[i]);

                if (DRAW_TRIANGLES) 
                {
                    glm::vec3 colr;
                    switch (i) {
                    case TRI_TYPE_BUILDING: colr = glm::vec3(0.5f, 0.5f, 0.5f); break;
                    case TRI_TYPE_HIGHWAY:  colr = glm::vec3(0.1f, 0.1f, 0.1f); break;
                    default: assert(false);
                        break;
                    }
                    glUniform3f(obj_color_loc, colr.r, colr.g, colr.b);
                    glDrawElements(GL_TRIANGLES, osmdata.tris[i].size() * 3, GL_UNSIGNED_INT, (void*)0);
                }
                if (DRAW_WIREFRAME)
                {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    glUniform3f(obj_color_loc, 1.0f, 0, 0);
                    glDrawElements(GL_TRIANGLES, osmdata.tris[i].size() * 3, GL_UNSIGNED_INT, (void*)0);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                }
            }
        }
        glBindVertexArray(0);

        light_shader.use();

        glUniformMatrix4fv(light_shader_Tview_loc, 1, GL_FALSE, glm::value_ptr(camera.view()));

        glBindVertexArray(VAO_light);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

        wnd.update();
    }

    //glDeleteVertexArrays(1, &VAO);
    //glDeleteBuffers(1, &VBO);
    //glDeleteBuffers(1, &EBO);

    return 0;
}