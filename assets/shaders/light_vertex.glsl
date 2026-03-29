#version 330 core
layout(location = 0) in vec3 v_pos;

uniform mat4 Tmodel;
uniform mat4 Tview;
uniform mat4 Tproj;

void main()
{
    gl_Position = Tproj * Tview * Tmodel * vec4(v_pos, 1.0);
}