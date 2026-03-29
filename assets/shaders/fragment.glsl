#version 330 core
out vec4 frag_color;

uniform vec3 light_pos;
uniform vec3 light_color;
uniform vec3 object_color;

in vec3 i_frag_normal;
in vec3 frag_pos;

void main()
{
    vec3 frag_normal = normalize(i_frag_normal);

    const float amb_strength = 0.5f;
    vec3 ambient = light_color * amb_strength;

    vec3 light_dir = normalize(light_pos - frag_pos);
    float diff = max(dot(frag_normal, light_dir), 0.0);
    vec3 diffuse = diff * light_color;

    vec3 result = (ambient + diffuse) * object_color;
    frag_color = vec4(result, 1.0);
} 