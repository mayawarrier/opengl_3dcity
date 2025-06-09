#ifndef VIEWPORT_HPP
#define VIEWPORT_HPP

#include <cstdint>
#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include "utils.hpp"



class window
{
public:
    struct fullscreen_t { explicit fullscreen_t() = default; };
    static constexpr fullscreen_t fullscreen;

public:
    window(const char* name, const char* title, ::size size) :
        window(name, title, size, false)
    {}

    window(const char* name, fullscreen_t) :
        window(name, name, { 0, 0 }, true)
    {}

    ~window() noexcept { cleanup(); }

    bool ok() const { return this->m_window && this->m_glcontext; }

    const char* name() const { return m_name; }

    size size() const {
        ::size s;
        SDL_GetWindowSize(m_window, &s.width, &s.height);
        return s;
    }

    float aspect_ratio() const {
        ::size s = size();
        return float(s.width) / s.height;
    }

    // Swap buffers and update screen.
    void update() { SDL_GL_SwapWindow(this->m_window); }

private:
    window(const char* name, const char* title, ::size size, bool fullscreen);

    void cleanup() noexcept;

private:
    const char* m_name;
    SDL_Window* m_window;
    SDL_GLContext m_glcontext;
};


// Blender-like camera.
class edit_camera
{
public:
    edit_camera();

    void new_frame();

    void process_event(const SDL_Event& event);

    // Update view after all events have been processed.
    void update();

    const glm::mat4& view() const noexcept { return m_last_view; }

private:
    // always up-to-date
    float m_dist;
    float m_yaw, m_pitch;

    // values in current frame
    glm::vec3 m_cam_x;
    glm::vec3 m_cam_y;
    glm::vec3 m_lookat;
    bool m_enable_move;

    // values from previous frame,
    // updated on update_view()
    glm::vec3 m_last_pos;
    glm::vec3 m_last_lookat;
    glm::vec3 m_last_up;
    glm::mat4 m_last_view;
};

#endif