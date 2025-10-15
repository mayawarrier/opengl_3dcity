
#include <glm/gtc/matrix_transform.hpp>
#include "glad/glad.h"

#include "viewport.hpp"


struct fullscreen_info
{
    size<int> size;
    int disp_idx;
};

static bool get_fullscreen_info(fullscreen_info& out_info)
{
    SDL_Point mouse_pos;
    SDL_GetGlobalMouseState(&mouse_pos.x, &mouse_pos.y);

    for (int i = 0; i < SDL_GetNumVideoDisplays(); ++i)
    {
        SDL_Rect disp_bounds;
        if (SDL_GetDisplayBounds(i, &disp_bounds) == 0 && 
            SDL_PointInRect(&mouse_pos, &disp_bounds)) 
        {
            SDL_DisplayMode desired = { 0 };
            desired.w = disp_bounds.w;
            desired.h = disp_bounds.h;
            
            SDL_DisplayMode closest;
            if (SDL_GetClosestDisplayMode(i, &desired, &closest) == nullptr) {
                logERROR("SDL_GetClosestDisplayMode(): %s", SDL_GetError());
                return false;
            }

            out_info.size.width = closest.w;
            out_info.size.height = closest.h;
            out_info.disp_idx = i;
            return true;
        }
    }
    return false;
}

window::window(const char* name, const char* title, ::size<int> size, bool fullscreen) :
    m_name(name), m_window(nullptr), m_glcontext(nullptr)
{
    logMESSAGE("Initializing %s", name);

    constexpr gladGLversionStruct want_glversion = { 3, 3 };

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, want_glversion.major);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, want_glversion.minor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    int disp_idx = 0;
    uint32_t window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI;
    
    if (fullscreen)
    {
        fullscreen_info info;
        if (!get_fullscreen_info(info)) {
            logERROR("get_fullscreen_info() failed");
            return;
        }
        size = info.size;
        disp_idx = info.disp_idx;
        window_flags |= SDL_WINDOW_FULLSCREEN;

        logMESSAGE("Using display %d (%s) with size %d x %d", 
            disp_idx, SDL_GetDisplayName(disp_idx), size.width, size.height);
    }

    this->m_window = SDL_CreateWindow(title,
        SDL_WINDOWPOS_CENTERED_DISPLAY(disp_idx), 
        SDL_WINDOWPOS_CENTERED_DISPLAY(disp_idx), 
        size.width, size.height, window_flags);
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

void window::cleanup() noexcept
{
    if (this->m_glcontext) {
        SDL_GL_DeleteContext(this->m_glcontext);
        this->m_glcontext = nullptr;
    }
    if (this->m_window) {
        SDL_DestroyWindow(this->m_window);
        this->m_window = nullptr;
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
        if (event.key.keysym.scancode == SDL_SCANCODE_LCTRL) {
            m_enable_move = true;
        }
        break;

    case SDL_KEYUP:
        if (event.key.keysym.scancode == SDL_SCANCODE_LCTRL) {
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
        if (event.motion.state & SDL_BUTTON_LMASK)
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

