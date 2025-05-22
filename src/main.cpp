
#include <array>

#include <SDL2/SDL.h>
#include <SDL2/SDL_main.h>

#include "glad/glad.h"

#include "shader.hpp"
#include "utils.hpp"

struct window
{
    const char* name;
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
    if (!window_create(wnd, "MAIN_WINDOW", "3D City", 500, 500)) {
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

    float vertices[] = {
         0.5f,  0.5f, 0.0f,  // top right
         0.5f, -0.5f, 0.0f,  // bottom right
        -0.5f, -0.5f, 0.0f,  // bottom left
        -0.5f,  0.5f, 0.0f   // top left 
    };
    unsigned int indices[] = {
        0, 1, 3,   // first triangle
        1, 2, 3    // second triangle
    };
    
    unsigned VAO, VBO, EBO;
    
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    
    glBindVertexArray(VAO);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(0);

    int color_uniform_loc = glGetUniformLocation(shader_prog, "ourColor");

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

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        glUseProgram(shader_prog);

        float color = float((std::sin(double(SDL_GetTicks64()) / 1000) + 1.0) / 2.0);
        glUniform4f(color_uniform_loc, 0.f, color, 0.f, 1.f);

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);


        SDL_GL_SwapWindow(wnd.handle);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shader_prog);

    window_destroy(wnd);
    return 0;
}