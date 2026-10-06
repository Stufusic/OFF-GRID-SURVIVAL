#include "ribbon_renderer.hpp"
#include <cmath>

namespace survival::gui {

EllipseMeshGeometry RibbonRenderer::generateEllipseMesh(
    double center_e_m,
    double center_n_m,
    double semi_major_a_m,
    double semi_minor_b_m,
    double orientation_rad) noexcept {
    
    EllipseMeshGeometry geom{};
    geom.vertices.reserve(kEllipseSegments + 1);
    geom.indices.reserve(kEllipseSegments * 3);

    // Đỉnh tâm
    geom.vertices.push_back(Vertex2D{
        static_cast<float>(center_e_m),
        static_cast<float>(center_n_m),
        0.0F, 0.0F
    });

    const double cos_psi = std::cos(orientation_rad);
    const double sin_psi = std::sin(orientation_rad);
    const double d_theta = (2.0 * 3.14159265358979323846) / static_cast<double>(kEllipseSegments);

    for (size_t i = 0; i < kEllipseSegments; ++i) {
        const double theta = static_cast<double>(i) * d_theta;
        const double local_x = semi_major_a_m * std::cos(theta);
        const double local_y = semi_minor_b_m * std::sin(theta);

        // Xoay theo góc phương vị orientation_rad và dịch tâm
        const double world_x = center_e_m + (local_x * cos_psi - local_y * sin_psi);
        const double world_y = center_n_m + (local_x * sin_psi + local_y * cos_psi);

        const float u = static_cast<float>(std::cos(theta));
        const float v = static_cast<float>(std::sin(theta));

        geom.vertices.push_back(Vertex2D{
            static_cast<float>(world_x),
            static_cast<float>(world_y),
            u, v
        });
    }

    // Nối các tam giác rẻ quạt từ tâm
    for (size_t i = 0; i < kEllipseSegments; ++i) {
        const uint16_t idx1 = static_cast<uint16_t>(i + 1);
        const uint16_t idx2 = static_cast<uint16_t>((i + 1) % kEllipseSegments + 1);

        geom.indices.push_back(0);
        geom.indices.push_back(idx1);
        geom.indices.push_back(idx2);
    }

    return geom;
}

RibbonMeshGeometry RibbonRenderer::generateRibbonStrip(
    const std::vector<EastingNorthingAlt>& waypoints,
    double ribbon_width_m) noexcept {
    
    RibbonMeshGeometry geom{};
    const size_t N = waypoints.size();
    if (N < 2) {
        return geom;
    }

    geom.vertices.reserve(N * 2);
    geom.indices.reserve((N - 1) * 6);
    const double half_w = ribbon_width_m * 0.5;

    for (size_t i = 0; i < N; ++i) {
        double dx = 0.0;
        double dy = 1.0;

        if (i + 1 < N) {
            dx = waypoints[i + 1].easting_m - waypoints[i].easting_m;
            dy = waypoints[i + 1].northing_m - waypoints[i].northing_m;
        } else if (i > 0) {
            dx = waypoints[i].easting_m - waypoints[i - 1].easting_m;
            dy = waypoints[i].northing_m - waypoints[i - 1].northing_m;
        }

        const double len = std::sqrt(dx * dx + dy * dy);
        double nx = -dy;
        double ny = dx;
        if (len > 1e-6) {
            nx /= len;
            ny /= len;
        }

        // Đỉnh bên trái (Left: u = -1.0)
        const double lx = waypoints[i].easting_m + nx * half_w;
        const double ly = waypoints[i].northing_m + ny * half_w;
        geom.vertices.push_back(Vertex2D{static_cast<float>(lx), static_cast<float>(ly), -1.0F, 0.0F});

        // Đỉnh bên phải (Right: u = +1.0)
        const double rx = waypoints[i].easting_m - nx * half_w;
        const double ry = waypoints[i].northing_m - ny * half_w;
        geom.vertices.push_back(Vertex2D{static_cast<float>(rx), static_cast<float>(ry), 1.0F, 0.0F});
    }

    // Sinh chỉ số tam giác (Indexed Triangles: 2 tam giác = 6 indices cho mỗi đoạn quad)
    for (size_t i = 0; i < N - 1; ++i) {
        const uint16_t l0 = static_cast<uint16_t>(2 * i);
        const uint16_t r0 = static_cast<uint16_t>(2 * i + 1);
        const uint16_t l1 = static_cast<uint16_t>(2 * (i + 1));
        const uint16_t r1 = static_cast<uint16_t>(2 * (i + 1) + 1);

        // Tam giác 1: (L0, R0, L1)
        geom.indices.push_back(l0);
        geom.indices.push_back(r0);
        geom.indices.push_back(l1);

        // Tam giác 2: (R0, R1, L1)
        geom.indices.push_back(r0);
        geom.indices.push_back(r1);
        geom.indices.push_back(l1);
    }

    return geom;
}

} // namespace survival::gui
