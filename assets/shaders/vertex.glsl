#version 330 core
layout (location = 0) in vec3 v_pos;
layout (location = 1) in vec3 v_normal;

uniform mat4 Tmodel;
uniform mat4 Tview;
uniform mat4 Tproj;
uniform mat3 Tnormal;

out vec3 i_frag_normal;
out vec3 frag_pos;

void main()
{
    gl_Position = Tproj * Tview * Tmodel * vec4(v_pos, 1.0);
    frag_pos = vec3(Tmodel * vec4(v_pos, 1.0));
    i_frag_normal = Tnormal * v_normal;
}