#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include "survival_core/types.hpp"

namespace survival::gui {

struct Vertex2D {
    float x{0.0F};
    float y{0.0F};
    float u{0.0F};
    float v{0.0F};
};

struct EllipseMeshGeometry {
    std::vector<Vertex2D> vertices{};
    std::vector<uint16_t> indices{};
};

struct RibbonMeshGeometry {
    std::vector<Vertex2D> vertices{}; // Dải đỉnh (Left & Right margin)
    std::vector<uint16_t> indices{};  // Danh sách chỉ số tam giác 6*(N-1)
};

class RibbonRenderer {
public:
    static constexpr size_t kEllipseSegments = 64;

    RibbonRenderer() noexcept = default;

    /**
     * @brief Sinh lưới vành khuyên Elip xác suất CE95 (Ring Mesh 64 đỉnh)
     */
    [[nodiscard]] static EllipseMeshGeometry generateEllipseMesh(
        double center_e_m,
        double center_n_m,
        double semi_major_a_m,
        double semi_minor_b_m,
        double orientation_rad
    ) noexcept;

    /**
     * @brief Sinh dải Indexed Triangles vệt ruy-băng bất định PDR (±sigma_L)
     */
    [[nodiscard]] static RibbonMeshGeometry generateRibbonStrip(
        const std::vector<EastingNorthingAlt>& waypoints,
        double ribbon_width_m
    ) noexcept;
};

} // namespace survival::gui
