#ifndef GLUTILS_HPP

#include "glad/glad.h"
#include "utils.hpp"


bool gl_load_shader(const fs::path& path, GLenum shader_type, unsigned& out_shader);

bool gl_make_program(const unsigned* shaders, int num_shaders, unsigned& out_program);

struct shaderinfo
{
    fs::path path;
    GLenum type;
};
bool gl_load_program(const shaderinfo* shaders, int num_shaders, unsigned& out_program);


// add texture params later (probably from ini file)

bool gl_load_texture2d(const fs::path& path, unsigned& out_texture);


#endif