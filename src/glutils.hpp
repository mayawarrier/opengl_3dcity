#ifndef GLUTILS_HPP

#include "glad/glad.h"
#include "utils.hpp"


#define GL_CLASS(classname, handlename)                                             \
    classname(const classname&) = delete;                                           \
    classname& operator=(const classname&) = delete;                                \
                                                                                    \
    classname(classname&& rhs) noexcept :                                           \
        handlename(std::exchange(rhs.handlename, 0))                                \
    {}                                                                              \
    classname& operator=(classname&& rhs) noexcept {                                \
        if (this != &rhs) { handlename = std::exchange(rhs.handlename, 0); }        \
        return *this;                                                               \
    }                                                                               \
    bool ok() const noexcept { return handlename != 0; }                            \
    unsigned handle() const noexcept { return handlename; }


class texture2d
{
public:
    texture2d(const fs::path& path);

    GL_CLASS(texture2d, m_handle)

    ~texture2d() noexcept;

private:
    unsigned m_handle;
};

class shaderstage
{
public:
    struct info
    {
        fs::path path;
        GLenum type;
    };

public:
    shaderstage(const fs::path& path, GLenum type);
    shaderstage(const info& info) : 
        shaderstage(info.path, info.type) 
    {}

    GL_CLASS(shaderstage, m_handle)

    ~shaderstage() noexcept;

private:
    unsigned m_handle;
};

class shader
{
public:
    shader(const shaderstage::info* stages, int num_stages);
    shader(const shaderstage* stages, int num_stages);

    GL_CLASS(shader, m_handle);

    int get_uniform_loc(const char* name) {
        return glGetUniformLocation(m_handle, name);
    }

    void bind_texture(int location, int texunit, const texture2d& tex) {
        glActiveTexture(GL_TEXTURE0 + texunit);
        glBindTexture(GL_TEXTURE_2D, tex.handle());
        glUniform1i(location, texunit);
    }

    void use() { glUseProgram(m_handle); }
    
    ~shader() noexcept;

private:
    unsigned m_handle;
};

#endif