#pragma once

#include <concepts>
#include <cmath>
#include "survival_core/types.hpp"

namespace survival::math {

template <typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

/**
 * @brief Chuẩn hóa góc về khoảng [-PI, PI] sử dụng atan2(sin, cos)
 * Tránh hoàn toàn lỗi modulus trên số thực có dấu
 */
template <FloatingPoint T>
[[nodiscard]] inline T normalizeAngleRad(T rad) noexcept {
    return static_cast<T>(std::atan2(std::sin(rad), std::cos(rad)));
}

/**
 * @brief Chuẩn hóa góc phương vị (Heading) về khoảng [0, 2*PI)
 * 0 = North, PI/2 = East, PI = South, 3*PI/2 = West
 */
template <FloatingPoint T>
[[nodiscard]] inline T normalizeHeadingRad(T rad) noexcept {
    T a = normalizeAngleRad(rad);
    if (a < static_cast<T>(0)) {
        a += static_cast<T>(kTwoPi);
    }
    return a;
}

/**
 * @brief Chuẩn hóa góc độ về khoảng [-180, 180]
 */
template <FloatingPoint T>
[[nodiscard]] inline T normalizeAngleDeg(T deg) noexcept {
    const T rad = deg * static_cast<T>(kDegToRad);
    return normalizeAngleRad(rad) * static_cast<T>(kRadToDeg);
}

/**
 * @brief Chuẩn hóa góc phương vị độ về khoảng [0, 360)
 */
template <FloatingPoint T>
[[nodiscard]] inline T normalizeHeadingDeg(T deg) noexcept {
    const T rad = deg * static_cast<T>(kDegToRad);
    return normalizeHeadingRad(rad) * static_cast<T>(kRadToDeg);
}

/**
 * @brief Chuyển đổi từ Degrees sang Radians
 */
template <FloatingPoint T>
[[nodiscard]] constexpr T degToRad(T deg) noexcept {
    return deg * static_cast<T>(kDegToRad);
}

/**
 * @brief Chuyển đổi từ Radians sang Degrees
 */
template <FloatingPoint T>
[[nodiscard]] constexpr T radToDeg(T rad) noexcept {
    return rad * static_cast<T>(kRadToDeg);
}

/**
 * @brief Tính sai lệch góc nhỏ nhất giữa hai góc phương vị [rad] trong [-PI, PI]
 */
template <FloatingPoint T>
[[nodiscard]] inline T angleDifferenceRad(T target_rad, T current_rad) noexcept {
    return normalizeAngleRad(target_rad - current_rad);
}

} // namespace survival::math
