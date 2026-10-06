#pragma once

#include <cmath>
#include <algorithm>
#include "survival_core/types.hpp"
#include "survival_core/math/angle_utils.hpp"

namespace survival::math {

// Giá trị tới hạn phân phối Chi-Square (Degrees of freedom = 2 và 3)
inline constexpr double kChiSquare2D_50 = 1.38629436112; // Chi2(2, p=0.50) -> CE50 multiplier: sqrt(1.3863) ≈ 1.1774
inline constexpr double kChiSquare2D_95 = 5.99146454711; // Chi2(2, p=0.95) -> CE95 multiplier: sqrt(5.9915) ≈ 2.4477
inline constexpr double kChiSquare3D_95 = 7.81472790325; // Chi2(3, p=0.95)

struct Ellipse2D {
    double semi_major_m{0.0};       // Bán trục lớn a [m]
    double semi_minor_m{0.0};       // Bán trục nhỏ b [m]
    double orientation_rad{0.0};    // Góc hướng của trục lớn so với hướng Bắc [rad] (0 = North, East-positive)
    double area_m2{0.0};            // Diện tích elip pi * a * b [m^2]
};

/**
 * @brief Tính toán hình học Elip bất định từ ma trận hiệp phương sai 2D [[var_e, cov_en], [cov_en, var_n]]
 * Kèm cơ chế chống suy biến 1D: lambda_2 >= 1e-6
 */
[[nodiscard]] inline Ellipse2D computeUncertaintyEllipse(
    double var_e,
    double var_n,
    double cov_en,
    double chi_square_factor = kChiSquare2D_95) noexcept
{
    // Bảo vệ hiệp phương sai không âm trên đường chéo
    const double v_e = std::max(var_e, 1e-8);
    const double v_n = std::max(var_n, 1e-8);

    // Tính Trị riêng (Eigenvalues) của ma trận 2x2 đối xứng:
    // lambda^2 - (v_e + v_n)*lambda + (v_e*v_n - cov_en^2) = 0
    const double trace = v_e + v_n;
    const double diff = v_e - v_n;
    const double disc = std::sqrt(std::max(0.0, diff * diff + 4.0 * cov_en * cov_en));

    double lambda_1 = 0.5 * (trace + disc);
    double lambda_2 = 0.5 * (trace - disc);

    // Chặn suy biến 1D theo khuyến nghị kỹ thuật
    lambda_1 = std::max(lambda_1, 1e-6);
    lambda_2 = std::max(lambda_2, 1e-6);

    const double a = std::sqrt(chi_square_factor * lambda_1);
    const double b = std::sqrt(chi_square_factor * lambda_2);

    // Góc phương vị trục lớn (Orientation relative to True North):
    // Trong hệ ENU: trục X là East, trục Y là North.
    // Góc xoay của eigenvector chính so với trục North:
    // angle = 0.5 * atan2(2 * cov_en, var_n - var_e)
    double orientation_rad = 0.5 * std::atan2(2.0 * cov_en, v_n - v_e);
    orientation_rad = normalizeHeadingRad(orientation_rad);

    const double area = kPi * a * b;

    return Ellipse2D{
        .semi_major_m = a,
        .semi_minor_m = b,
        .orientation_rad = orientation_rad,
        .area_m2 = area
    };
}

/**
 * @brief Phép biến đổi NGHỊCH ĐẢO của computeUncertaintyEllipse:
 *        elip tin cậy (a, b, heading) -> hiệp phương sai 2D [[var_e, cov_en], [cov_en, var_n]].
 *        heading_rad đo từ hướng Bắc, chiều Đông-dương (cùng quy ước computeUncertaintyEllipse).
 *        Dùng để cấp đúng đơn vị PHƯƠNG SAI [m^2] cho NEES/NIS (audit H3: cấm truyền bán trục [m]).
 */
[[nodiscard]] inline std::array<double, 4> ellipseToCov2D(
    double semi_major_m,
    double semi_minor_m,
    double heading_rad,
    double chi_square_factor = kChiSquare2D_95) noexcept
{
    const double k = std::max(chi_square_factor, 1e-9);
    const double lam1 = (semi_major_m * semi_major_m) / k;  // Phương sai dọc trục lớn
    const double lam2 = std::max((semi_minor_m * semi_minor_m) / k, 1e-8);
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);
    // Trục lớn hướng u = (sinθ, cosθ) trong hệ (East, North)
    const double var_e = lam1 * s * s + lam2 * c * c;
    const double var_n = lam1 * c * c + lam2 * s * s;
    const double cov_en = (lam1 - lam2) * s * c;
    return {var_e, cov_en, cov_en, var_n};
}

} // namespace survival::math
