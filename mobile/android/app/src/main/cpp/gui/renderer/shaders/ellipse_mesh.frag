#version 300 es
precision mediump float;

in vec2 v_local_uv;
uniform vec4 u_color;
uniform float u_tactical_red_mode;

out vec4 fragColor;

void main() {
    float dist_sq = dot(v_local_uv, v_local_uv);
    if (dist_sq > 1.0) {
        discard;
    }

    float edge_alpha = smoothstep(0.85, 1.0, dist_sq) * 0.8;
    float fill_alpha = (1.0 - dist_sq) * 0.15;
    float alpha = edge_alpha + fill_alpha;

    vec4 final_color = u_color;
    if (u_tactical_red_mode > 0.5) {
        final_color = vec4(final_color.r * 0.9 + 0.1, 0.0, 0.0, 1.0);
    }

    fragColor = vec4(final_color.rgb, alpha * final_color.a);
}
