#ifndef SHADER_HPP

#include "utils.hpp"


bool gl_load_shader(const fs::path& path, GLenum shader_type, unsigned& out_shader);

bool gl_create_program(const unsigned* shaders, int num_shaders, unsigned& out_program);


struct shaderinfo
{
    fs::path path;
    GLenum type;
};
bool gl_load_program(const shaderinfo* shaders, int num_shaders, unsigned& out_program);



#endif