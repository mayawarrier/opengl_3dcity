
#include <string>
#include "glad/glad.h"

#define STBI_WINDOWS_UTF8 1
#define STB_IMAGE_IMPLEMENTATION 1
#include "stb/stb_image.h"

#include "glutils.hpp"


static bool gl_load_shader(const fs::path& path, GLenum shader_type, unsigned& out_shader)
{
    size_t filesize;
    std::unique_ptr<char[]> filedata;
    if (!read_file(path, filedata, filesize)) {
        return false;
    }
    const char* shader_src = filedata.get();
    const int shader_srclen = int(filesize);

    unsigned shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, &shader_src, &shader_srclen);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        logERROR("Failed to compile shader %s", path.string().c_str());

        int log_bufsize = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_bufsize);
        if (log_bufsize > 0)
        {
            std::string log; log.resize(log_bufsize - 1);
            glGetShaderInfoLog(shader, log_bufsize, nullptr, log.data());
            logMESSAGE("Shader compile log: %s", log.c_str());
        }

        glDeleteShader(shader);
        return false;
    }

    out_shader = shader;
    return true;
}

// Always detach shaders or delete the program after this call!
static bool gl_link_program(unsigned program)
{
    glLinkProgram(program);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        logERROR("Failed to link shader program");

        int log_bufsize = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_bufsize);
        if (log_bufsize > 0)
        {
            std::string log; log.resize(log_bufsize - 1);
            glGetProgramInfoLog(program, log_bufsize, nullptr, log.data());
            logMESSAGE("Program link log: %s", log.c_str());
        }
        return false;
    }
    return true;
}

static void gl_delete_all_shaders(unsigned program, const unsigned* shaders, int num_shaders)
{
    for (int i = 0; i < num_shaders; ++i) {
        glDetachShader(program, shaders[i]);
        glDeleteShader(shaders[i]);
    }
}

static bool gl_make_program(const unsigned* shaders, int num_shaders, unsigned& out_program)
{
    unsigned program = glCreateProgram();
    for (int i = 0; i < num_shaders; ++i) {
        glAttachShader(program, shaders[i]);
    }

    if (!gl_link_program(program)) {
        glDeleteProgram(program);
        return false;
    }
    for (int i = 0; i < num_shaders; ++i) {
        glDetachShader(program, shaders[i]);
    }

    out_program = program;
    return true;
}

static bool gl_load_program(const shaderfile::info* shader_info, int num_shaders, unsigned& out_program)
{
    unsigned program = glCreateProgram();

    auto shaders = std::make_unique<unsigned[]>(num_shaders);
    for (int i = 0; i < num_shaders; ++i) 
    {
        if (!gl_load_shader(shader_info[i].path, shader_info[i].type, shaders[i]))
        {
            gl_delete_all_shaders(program, shaders.get(), i);
            glDeleteProgram(program);
            return false;
        }
        glAttachShader(program, shaders[i]);
    }

    if (!gl_link_program(program)) {
        glDeleteProgram(program);
        return false;
    }
    gl_delete_all_shaders(program, shaders.get(), num_shaders);
    
    out_program = program;
    return true;
}

// todo: perf: if this is called 100-1000s of times, it would be better
// to glGen all the textures upfront, instead of one at a time 
static bool gl_load_texture2d(const fs::path& path, unsigned& out_texture)
{
    unsigned texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    auto pathstr = path.string();

    int actual_channels;
    if (!stbi_info(pathstr.c_str(), nullptr, nullptr, &actual_channels)) {
        logERROR("stbi_info() failed for file %s: %s", pathstr.c_str(), stbi_failure_reason());
        glDeleteTextures(1, &texture);
        return false;
    }

    int width, height;
    int load_channels = std::clamp(actual_channels, 3, 4);
    
    unsigned char* image;
    stbi_set_flip_vertically_on_load(1);
    {
        image = stbi_load(pathstr.c_str(), &width, &height, nullptr, load_channels);
        if (!image) {
            logERROR("stbi_load() failed for file %s: %s", pathstr.c_str(), stbi_failure_reason());
            stbi_set_flip_vertically_on_load(0);
            glDeleteTextures(1, &texture);
            return false;
        }
    }
    stbi_set_flip_vertically_on_load(0);

    GLenum texformat, imgformat;
    if (load_channels == 3) {
        texformat = GL_RGB8;
        imgformat = GL_RGB;
    } else {
        assert(load_channels == 4);
        texformat = GL_RGBA8;
        imgformat = GL_RGBA;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, texformat, width, height, 0, imgformat, GL_UNSIGNED_BYTE, image);
    glGenerateMipmap(GL_TEXTURE_2D); // all levels

    stbi_image_free(image);

    out_texture = texture;
    return true;
}

shaderfile::shaderfile(const fs::path& path, GLenum type) : m_handle(0)
{
    gl_load_shader(path, type, m_handle);
}

shaderfile::~shaderfile() noexcept { glDeleteShader(m_handle); }

shader::shader(const shaderfile::info* files, int num_files) : m_handle(0)
{
    gl_load_program(files, num_files, m_handle);
}

shader::shader(const shaderfile* files, int num_files) : m_handle(0)
{
    // todo: cast here is bad, obviously
    gl_make_program((const unsigned*)files, num_files, m_handle);
}

shader::~shader() noexcept { glDeleteProgram(m_handle); }

texture2d::texture2d(const fs::path& path) : m_handle(0)
{
    gl_load_texture2d(path, m_handle);
}

texture2d::~texture2d() noexcept { glDeleteTextures(1, &m_handle); }
