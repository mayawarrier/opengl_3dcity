#ifndef GLUTILS_HPP

#include "glad/glad.h"
#include "utils.hpp"

class texture2d
{
public:
    texture2d(const fs::path& path);

    HANDLE_CLASS(texture2d, m_handle, 0)

    unsigned handle() const noexcept { return m_handle; }

    ~texture2d() noexcept;

private:
    unsigned m_handle;
};

class shaderfile
{
public:
    struct info
    {
        fs::path path;
        GLenum type;
    };

public:
    shaderfile(const fs::path& path, GLenum type);
    shaderfile(const info& info) : 
        shaderfile(info.path, info.type) 
    {}

    HANDLE_CLASS(shaderfile, m_handle, 0)

    ~shaderfile() noexcept;

private:
    unsigned m_handle;
};

class shader
{
public:
    shader(const shaderfile::info* files, int num_files);
    shader(const shaderfile* files, int num_files);

    HANDLE_CLASS(shader, m_handle, 0);

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