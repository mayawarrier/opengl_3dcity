
#include <string>
#include "glad/glad.h"

#include "shader.hpp"


bool gl_load_shader(const fs::path& path, GLenum shader_type, unsigned& out_shader)
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

bool gl_create_program(const unsigned* shaders, int num_shaders, unsigned& out_program)
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

bool gl_load_program(const shaderinfo* shader_info, int num_shaders, unsigned& out_program)
{
    unsigned program = glCreateProgram();

    auto shaders = std::make_unique<unsigned[]>(num_shaders);
    for (int i = 0; i < num_shaders; ++i) 
    {
        if (!gl_load_shader(shader_info[i].path, shader_info[i].type, shaders[i]))
        {
            for (int j = 0; j < i; ++j) {
                glDetachShader(program, shaders[j]);
                glDeleteShader(shaders[j]);
            }
            glDeleteProgram(program);
            return false;
        }

        glAttachShader(program, shaders[i]);
    }

    if (!gl_link_program(program)) {
        glDeleteProgram(program);
        return false;
    }

    for (int i = 0; i < num_shaders; ++i) {
        glDetachShader(program, shaders[i]);
        glDeleteShader(shaders[i]);
    }
    out_program = program;
    return true;
}