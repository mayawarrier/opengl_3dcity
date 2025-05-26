#version 330 core
out vec4 frag_color;

in vec2 texcoord;

uniform sampler2D texture0;
uniform sampler2D texture1;

void main()
{
    frag_color = mix(texture(texture0, texcoord), texture(texture1, texcoord), 0.35);
} 