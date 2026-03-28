#version 330 core
layout (location = 0) in vec3 v_pos;
layout (location = 1) in vec3 v_normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 frag_normal;
out vec3 frag_pos;

void main()
{
    gl_Position = projection * view * model * vec4(v_pos, 1.0);
    frag_pos = (model * vec4(v_pos, 1.0)).xyz;
    frag_normal = v_normal;
}