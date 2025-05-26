#version 330 core
layout (location = 0) in vec3 v_pos;
layout (location = 1) in vec2 v_texcoord;

out vec2 texcoord;

//uniform mat4 transform;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    //gl_Position = transform * vec4(v_pos, 1.0);
    gl_Position = projection * view * model * vec4(v_pos, 1.0);
    texcoord = v_texcoord;
}