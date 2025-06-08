
#include <glm/gtc/matrix_transform.hpp>
#include "glad/glad.h"

#include "viewport.hpp"

window::window(const char* name, const char* title, int width, int height) :
    m_name(name), m_window(nullptr), m_glcontext(nullptr), m_initwidth(width), m_initheight(height)
{
    logMESSAGE("Initializing %s", name);

    constexpr gladGLversionStruct want_glversion = { 3, 3 };

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, want_glversion.major);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, want_glversion.minor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    this->m_window = SDL_CreateWindow(title,
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_OPENGL);
    if (!this->m_window) {
        logERROR("SDL_CreateWindow(): %s", SDL_GetError());
        cleanup();
        return;
    }

    this->m_glcontext = SDL_GL_CreateContext(this->m_window);
    if (!this->m_glcontext) {
        logERROR("SDL_GL_CreateContext(): %s", SDL_GetError());
        cleanup();
        return;
    }
    if (SDL_GL_SetSwapInterval(1) != 0) {
        logERROR("SDL_GL_SetSwapInterval(): %s", SDL_GetError());
        cleanup();
        return;
    }

    // todo: on Windows, GL func pointers are only valid
    // for the context they were loaded in. If multiple windows
    // are required, I might need glad2 to get multi-context support
    gladLoadGLLoader(SDL_GL_GetProcAddress);
    logMESSAGE("OpenGL Vendor:   %s", glGetString(GL_VENDOR));
    logMESSAGE("OpenGL Renderer: %s", glGetString(GL_RENDERER));
    logMESSAGE("OpenGL Version:  %s", glGetString(GL_VERSION));

    if (GLVersion.major < want_glversion.major ||
        (GLVersion.major == want_glversion.major && GLVersion.minor < want_glversion.minor)) {
        logERROR("OpenGL version must be at least %d.%d", want_glversion.major, want_glversion.minor);
        cleanup();
        return;
    }
}

bool window::set_fullscreen(bool enable)
{
    if (enable) 
    {
        logMESSAGE("Switching %s to fullscreen mode", m_name);

        int disp_idx = SDL_GetWindowDisplayIndex(m_window);
        if (disp_idx < 0) {
            logERROR("SDL_GetWindowDisplayIndex(): %s", SDL_GetError());
            return false;
        }
        SDL_Rect disp_bounds;
        if (SDL_GetDisplayBounds(disp_idx, &disp_bounds) != 0) {
            logERROR("SDL_GetDisplayBounds(): %s", SDL_GetError());
            return false;
        }

        SDL_DisplayMode desired = { 0 };
        desired.w = disp_bounds.w;
        desired.h = disp_bounds.h;

        SDL_DisplayMode closest;
        if (SDL_GetClosestDisplayMode(disp_idx, &desired, &closest) == nullptr) {
            logERROR("SDL_GetClosestDisplayMode(): %s", SDL_GetError());
            return false;
        }

        SDL_SetWindowSize(m_window, closest.w, closest.h);
        SDL_SetWindowFullscreen(m_window, SDL_WINDOW_FULLSCREEN);
        glViewport(0, 0, closest.w, closest.h);

        return true;
    }
    else {
        logMESSAGE("Switching %s to windowed mode", m_name);

        SDL_SetWindowSize(m_window, m_initwidth, m_initheight);
        SDL_SetWindowFullscreen(m_window, 0);
        glViewport(0, 0, m_initwidth, m_initheight);

        return true;
    }
}

void window::cleanup() noexcept
{
    if (this->m_glcontext) {
        SDL_GL_DeleteContext(this->m_glcontext);
    }
    if (this->m_window) {
        SDL_DestroyWindow(this->m_window);
    }
}

edit_camera::edit_camera() :
    m_dist(10),
    m_yaw(90),
    m_pitch(0),
    m_cam_x(0),
    m_cam_y(0),
    m_lookat(0),
    m_enable_move(false),
    m_last_pos(0, 0, m_dist),
    m_last_lookat(0, 0, 0),
    m_last_up(0, 1, 0),
    m_last_view(0)
{}

void edit_camera::new_frame()
{
    glm::vec3 lookdir = m_last_lookat - m_last_pos;
    m_cam_x = glm::normalize(glm::cross(lookdir, m_last_up));
    m_cam_y = glm::normalize(glm::cross(lookdir, m_cam_x));
    m_lookat = m_last_lookat;
}

void edit_camera::process_event(const SDL_Event& event)
{
    switch (event.type)
    {
    case SDL_KEYDOWN:
        if (event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) {
            m_enable_move = true;
        }
        break;

    case SDL_KEYUP:
        if (event.key.keysym.scancode == SDL_SCANCODE_LSHIFT) {
            m_enable_move = false;
        }
        break;

    case SDL_MOUSEWHEEL:
    {
        const float sensitivity = 0.15f;
        m_dist -= sensitivity * m_dist * event.wheel.y;
        m_dist = std::max(0.01f, m_dist); // 0 causes lookAt to fail
        break;
    }

    case SDL_MOUSEMOTION:
        if (event.motion.state & SDL_BUTTON_MMASK)
        {
            if (m_enable_move)
            {
                const float sensitivity = 1.f;
                glm::vec3 incr = -sensitivity * (
                    (float(event.motion.xrel) * m_cam_x) +
                    (float(event.motion.yrel) * m_cam_y));

                m_lookat += incr;
            }
            else {
                const float sensitivity = 0.125f;
                m_yaw += sensitivity * event.motion.xrel;
                m_pitch -= sensitivity * event.motion.yrel;

                m_pitch = wrap_angle(m_pitch);
                m_yaw = wrap_angle(m_yaw);
            }
        }
        break;

    default: break;
    }
}

void edit_camera::update()
{
    if ((m_pitch >= 0 && m_pitch < 90.0f) || (m_pitch > 270.f && m_pitch < 360.f)) {
        m_last_up = glm::vec3(0, 1, 0);
    }
    else if (m_pitch >= 90.0f && m_pitch <= 270.f) {
        m_last_up = glm::vec3(0, -1, 0);
    }

    glm::vec3 cam_dir(
        std::cos(glm::radians(m_yaw)) * std::cos(glm::radians(m_pitch)),
        -std::sin(glm::radians(m_pitch)),
        std::sin(glm::radians(m_yaw)) * std::cos(glm::radians(m_pitch)));

    m_last_pos = m_lookat + m_dist * glm::normalize(cam_dir);
    m_last_lookat = m_lookat;

    m_last_view = glm::lookAt(m_last_pos, m_last_lookat, m_last_up);
}

