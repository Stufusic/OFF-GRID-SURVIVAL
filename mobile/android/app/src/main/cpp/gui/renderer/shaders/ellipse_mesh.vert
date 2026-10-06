#version 300 es
layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_uv;

uniform mat4 u_mvp;

out vec2 v_local_uv;

void main() {
    v_local_uv = a_uv;
    gl_Position = u_mvp * vec4(a_position, 0.0, 1.0);
}
