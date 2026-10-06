#pragma once

#include "survival_core/types.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace survival::math {

/**
 * @brief Chuyển đổi Quaternion q_nb (Body -> Nav) sang ma trận xoay DCM 3x3 R_nb
 * Thỏa mãn: v_nav = R_nb * v_body
 */
[[nodiscard]] inline Eigen::Matrix3d quaternionToRotationMatrix(const Quaterniond& q) noexcept {
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    Eigen::Matrix3d R;
    R(0, 0) = 1.0 - 2.0 * (y * y + z * z);
    R(0, 1) = 2.0 * (x * y - w * z);
    R(0, 2) = 2.0 * (x * z + w * y);

    R(1, 0) = 2.0 * (x * y + w * z);
    R(1, 1) = 1.0 - 2.0 * (x * x + z * z);
    R(1, 2) = 2.0 * (y * z - w * x);

    R(2, 0) = 2.0 * (x * z - w * y);
    R(2, 1) = 2.0 * (y * z + w * x);
    R(2, 2) = 1.0 - 2.0 * (x * x + y * y);

    return R;
}

/**
 * @brief Xoay một vector 3D từ hệ Body sang hệ Navigation (ENU) bằng Quaternion q_nb
 */
[[nodiscard]] inline Vector3d rotateBodyToNav(const Quaterniond& q, const Vector3d& v_body) noexcept {
    const Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    const Eigen::Vector3d vb(v_body.x, v_body.y, v_body.z);
    const Eigen::Vector3d vn = R * vb;
    return Vector3d{vn.x(), vn.y(), vn.z()};
}

/**
 * @brief Nhân hai Quaternion: q_out = q1 ⊗ q2
 */
[[nodiscard]] inline Quaterniond quaternionMultiply(const Quaterniond& q1, const Quaterniond& q2) noexcept {
    return Quaterniond{
        .w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        .x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        .y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        .z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    };
}

/**
 * @brief Tạo Quaternion từ góc Euler RPY (Roll quanh X, Pitch quanh Y, Yaw quanh Z theo chiều kim đồng hồ)
 */
[[nodiscard]] inline Quaterniond eulerToQuaternion(double roll_rad, double pitch_rad, double yaw_rad) noexcept {
    // Yaw la bàn quay quanh Z theo chiều kim đồng hồ từ Bắc (+Y) -> Đông (+X)
    // Tương ứng với phép quay góc -yaw quanh trục +Z
    const double cr = std::cos(roll_rad * 0.5);
    const double sr = std::sin(roll_rad * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cy = std::cos(-yaw_rad * 0.5);
    const double sy = std::sin(-yaw_rad * 0.5);

    Quaterniond q{};
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

/**
 * @brief Trích xuất góc la bàn Compass Heading (0 = North, East = +pi/2, South = pi, West = 3pi/2)
 */
[[nodiscard]] inline double extractCompassHeadingRad(const Quaterniond& q) noexcept {
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;

    // Vector Forward thân máy [0, 1, 0]^T chiếu vào ENU có:
    // East component = R(0, 1) = 2*(x*y - w*z)
    // North component = R(1, 1) = 1 - 2*(x*x + z*z)
    const double east = 2.0 * (x * y - w * z);
    const double north = 1.0 - 2.0 * (x * x + z * z);
    double psi = std::atan2(east, north);
    if (psi < 0.0) {
        psi += kTwoPi;
    }
    return psi;
}

} // namespace survival::math
