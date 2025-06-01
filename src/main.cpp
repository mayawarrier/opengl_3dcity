
#include <array>

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

    shaderstage::info shader_stages[2] = {
        { "assets/shaders/vertex.vert", GL_VERTEX_SHADER },
        { "assets/shaders/fragment.frag", GL_FRAGMENT_SHADER }
    };
    shader shader(shader_stages, 2);
    if (!shader.ok()) {
        return -1;
    }

    texture2d containertex("assets/textures/container.jpg");
    if (!containertex.ok()) {
        return -1;
    }

    texture2d logotex("assets/textures/Opengl-logo.png");
    if (!logotex.ok()) {
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    float vertices[] = {
        -0.5f, -0.5f, -0.5f,   0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,   1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
         0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,   0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,   0.0f, 0.0f,
                               
        -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,   1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,   1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,   0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
                               
        -0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
                               
         0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
         0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
                               
        -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,   1.0f, 1.0f,
         0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,   1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,   0.0f, 1.0f,
                               
        -0.5f,  0.5f, -0.5f,   0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,   1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,   1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,   0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,   0.0f, 1.0f
    };
    
    unsigned VAO, VBO, EBO;
    
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    
    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        //glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    
        // positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // tex coords
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }
    glBindVertexArray(0);

    int texture0_loc = shader.get_uniform_loc("texture0");
    int texture1_loc = shader.get_uniform_loc("texture1");
    int mvp_loc = shader.get_uniform_loc("MVP");

    constexpr int num_cubes = 30;
    auto cubes = std::make_unique<std::pair<glm::vec3, glm::vec3>[]>(num_cubes);
    for (int i = 0; i < num_cubes; ++i)
    {
        auto pos_xz = glm::circularRand(3.0f);
        cubes[i].first = glm::vec3(pos_xz[0], 0.0f, pos_xz[1]);
        cubes[i].second = glm::sphericalRand(1.0f);
    }

    edit_camera camera;

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), wnd.aspect_ratio(), 0.001f, 100.0f);

    uint64_t last_ticks = 0;

    SDL_SetRelativeMouseMode(SDL_TRUE);

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

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        shader.use();
        shader.bind_texture(texture0_loc, 0, containertex);
        shader.bind_texture(texture1_loc, 1, logotex);

        glBindVertexArray(VAO);
        {
            for (int i = 0; i < num_cubes; ++i)
            {
                glm::mat4 model(1.0f);
                model = glm::translate(model, cubes[i].first);
                model = glm::rotate(model, float(time), cubes[i].second);
                model = glm::scale(model, glm::vec3(0.5f, 0.5f, 0.5f));

                glm::mat4 mvp = projection * camera.view() * model;
                glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));

                glDrawArrays(GL_TRIANGLES, 0, 36);
            }

            glm::mat4 model(1.0f);
            model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));

            glm::mat4 mvp = projection * camera.view() * model;
            glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        glBindVertexArray(0);

        wnd.update();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    return 0;
}