
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

struct window
{
    const char* name;
    int width, height; // assume this does not change
    SDL_Window* handle;
    SDL_GLContext glcontext;
};

void window_destroy(window& wnd)
{
    if (wnd.glcontext) {
        SDL_GL_DeleteContext(wnd.glcontext);
    }
    if (wnd.handle) {
        SDL_DestroyWindow(wnd.handle);
    }
}

bool window_create(window& wnd, const char* name, const char* title, int width, int height)
{
    wnd.name = name;
    wnd.width = width;
    wnd.height = height;

    logMESSAGE("Initializing %s", name);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    wnd.handle = SDL_CreateWindow(title, 
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_OPENGL);
    if (!wnd.handle) {
        logERROR("SDL_CreateWindow(): %s", SDL_GetError());
        window_destroy(wnd);
        return false;
    }

    wnd.glcontext = SDL_GL_CreateContext(wnd.handle);
    if (!wnd.glcontext) {
        logERROR("SDL_GL_CreateContext(): %s", SDL_GetError());
        window_destroy(wnd);
        return false;
    }

    // todo: on Windows, GL func pointers are only valid
    // for the context they were loaded in. If multiple windows
    // are required, I might need glad2 to get multi-context support
    gladLoadGLLoader(SDL_GL_GetProcAddress);
    logMESSAGE("OpenGL Vendor:   %s", glGetString(GL_VENDOR));
    logMESSAGE("OpenGL Renderer: %s", glGetString(GL_RENDERER));
    logMESSAGE("OpenGL Version:  %s", glGetString(GL_VERSION));

    if (SDL_GL_SetSwapInterval(1) != 0) {
        logERROR("SDL_GL_SetSwapInterval(): %s", SDL_GetError());
        window_destroy(wnd);
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

    window wnd;
    if (!window_create(wnd, "MAIN_WINDOW", "3D City", 750, 750)) {
        return -1;
    }

    shaderinfo shaders[2] = {
        { "assets/shaders/vertex.vert", GL_VERTEX_SHADER },
        { "assets/shaders/fragment.frag", GL_FRAGMENT_SHADER }
    };
    unsigned shader_prog;
    if (!gl_load_program(shaders, 2, shader_prog)) {
        window_destroy(wnd);
        return -1;
    }

    unsigned containertex;
    if (!gl_load_texture2d("assets/textures/container.jpg", containertex)) {
        glDeleteShader(shader_prog);
        window_destroy(wnd);
        return -1;
    }

    unsigned logotex;
    if (!gl_load_texture2d("assets/textures/Opengl-logo.png", logotex)) {
        glDeleteTextures(1, &containertex);
        glDeleteShader(shader_prog);
        window_destroy(wnd);
        return -1;
    }

    //float vertices[] = {
    //    // positions           // texture coords
    //     0.5f,  0.5f, 0.0f,    1.0f, 1.0f,  // top right
    //     0.5f, -0.5f, 0.0f,    1.0f, 0.0f,  // bottom right
    //    -0.5f, -0.5f, 0.0f,    0.0f, 0.0f,  // bottom left
    //    -0.5f,  0.5f, 0.0f,    0.0f, 1.0f,  // top left 
    //};
    //unsigned int indices[] = {
    //    0, 1, 3,   // first triangle
    //    1, 2, 3    // second triangle
    //};

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

    int texture0_loc = glGetUniformLocation(shader_prog, "texture0");
    int texture1_loc = glGetUniformLocation(shader_prog, "texture1");
    //int transform_loc = glGetUniformLocation(shader_prog, "transform");

    int model_loc = glGetUniformLocation(shader_prog, "model");
    int view_loc = glGetUniformLocation(shader_prog, "view");
    int proj_loc = glGetUniformLocation(shader_prog, "projection");

    //std::pair<glm::vec3, glm::vec3> cubes[] = {
    //    { glm::vec3(0.0f,  0.0f,  0.0f),   glm::sphericalRand(1.0f) },
    //    { glm::vec3(2.0f,  5.0f, -15.0f),  glm::sphericalRand(1.0f) },
    //    { glm::vec3(-1.5f, -2.2f, -2.5f),  glm::sphericalRand(1.0f) },
    //    { glm::vec3(-3.8f, -2.0f, -12.3f), glm::sphericalRand(1.0f) },
    //    { glm::vec3(2.4f, -0.4f, -3.5f),   glm::sphericalRand(1.0f) },
    //    { glm::vec3(-1.7f,  3.0f, -7.5f),  glm::sphericalRand(1.0f) },
    //    { glm::vec3(1.3f, -2.0f, -2.5f),   glm::sphericalRand(1.0f) },
    //    { glm::vec3(1.5f,  2.0f, -2.5f),   glm::sphericalRand(1.0f) },
    //    { glm::vec3(1.5f,  0.2f, -1.5f),   glm::sphericalRand(1.0f) },
    //    { glm::vec3(-1.3f,  1.0f, -1.5f),  glm::sphericalRand(1.0f) },
    //};

    constexpr int num_cubes = 30;
    auto cubes = std::make_unique<std::pair<glm::vec3, glm::vec3>[]>(num_cubes);
    for (int i = 0; i < num_cubes; ++i)
    {
        auto pos_xz = glm::circularRand(3.0f);
        cubes[i].first = glm::vec3(pos_xz[0], 0.0f, pos_xz[1]);
        cubes[i].second = glm::sphericalRand(1.0f);
    }

    //glm::mat4 init_model(1.0f);
    //init_model = glm::rotate(init_model, glm::radians(-55.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), float(wnd.width) / wnd.height, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

    bool quit = false;
    while (!quit)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                quit = true;
                break;

            default: break;
            }
        }

        double time = double(SDL_GetTicks64()) / 1000;
        float sin_curve = float((std::sin(time) + 1.0) / 2.0);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glUseProgram(shader_prog);

        //float color = sin_curve;
        //glUniform4f(color_uniform_loc, 0.f, color, 0.f, 1.f);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, containertex);
        glUniform1i(texture0_loc, 0);
        
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, logotex);
        glUniform1i(texture1_loc, 1);

        glBindVertexArray(VAO);
        {
            //glm::mat4 trans(1.0f);
            //trans = glm::translate(trans, glm::vec3(0.5f, -0.5f, 0.0f));
            //trans = glm::rotate(trans, float(time), glm::vec3(0.0f, 0.0f, 1.0));
            //
            //glUniformMatrix4fv(transform_loc, 1, GL_FALSE, glm::value_ptr(trans));
            //
            //glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
            //
            //glm::mat4 trans2(1.0f);
            //trans2 = glm::translate(trans2, glm::vec3(-0.5f, 0.5f, 0.0f));
            //trans2 = glm::scale(trans2, sin_curve * glm::vec3(1.0f, 1.0f, 1.0f));
            //
            //glUniformMatrix4fv(transform_loc, 1, GL_FALSE, glm::value_ptr(trans2));
            //
            //glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

            glm::mat4 view(1.0f);
            view = glm::rotate(view, -float(time), glm::normalize(glm::vec3(0.1f, 0.0f, 1.0f)));
            view = glm::translate(view, glm::vec3(0.0f, 0.0f, -10.0f));
            
            view = glm::rotate(view, glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            
            

            for (int i = 0; i < num_cubes; ++i)
            {
                glm::mat4 model(1.0f);
                model = glm::rotate(model, float(time), glm::vec3(0.0f, 1.0f, 0.0f));
                model = glm::translate(model, cubes[i].first);
                model = glm::rotate(model, float(time), cubes[i].second);
                model = glm::scale(model, glm::vec3(0.5f, 0.5f, 0.5f));
                //glm::vec3(1.0f, 1.0f, 0.0f));

                glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));
                glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(view));
                glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(projection));

                glDrawArrays(GL_TRIANGLES, 0, 36);
            }

            glm::mat4 model(1.0f);
            model = glm::scale(model, glm::vec3(2.0f, 2.0f, 2.0f));
            glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));
            glDrawArrays(GL_TRIANGLES, 0, 36);

            //glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);

        SDL_GL_SwapWindow(wnd.handle);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shader_prog);
    glDeleteTextures(1, &containertex);
    glDeleteTextures(1, &logotex);

    window_destroy(wnd);
    return 0;
}