#version 410 core
layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;
layout(location = 0) out vec4 f_Color;

uniform vec3  u_Color;
uniform vec3  u_LightDir;
uniform vec3  u_LightColor;
uniform vec3  u_AmbientColor;
uniform vec3  u_ViewPosition;
uniform float u_Shininess;
uniform bool  u_FlatShading;

void main() {
    vec3 N = u_FlatShading
        ? normalize(cross(dFdx(v_Position), dFdy(v_Position)))
        : normalize(v_Normal);
    vec3 L = normalize(u_LightDir);
    vec3 V = normalize(u_ViewPosition - v_Position);
    vec3 H = normalize(L + V);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), u_Shininess);

    vec3 ambient  = u_AmbientColor * u_Color;
    vec3 diffuse  = u_LightColor * u_Color * diff;
    vec3 specular = u_LightColor * spec;

    f_Color = vec4(ambient + diffuse + specular, 1.0);
}
