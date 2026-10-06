#version 300 es
precision mediump float;

in vec2 v_texcoord;
uniform sampler2D u_texture;

out vec4 fragColor;

void main() {
    vec4 src = texture(u_texture, v_texcoord);
    // Chuyển đổi thành dải phổ đỏ đơn sắc 650nm cho nhìn đêm (Night-Vision Tactical Red)
    float gray = dot(src.rgb, vec3(0.299, 0.587, 0.114));
    fragColor = vec4(gray * 1.2, 0.0, 0.0, src.a);
}
