#include <gtest/gtest.h>
#include "survival_core/math/frame_conventions.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::math;

TEST(QuaternionConventionsTest, PureYawPlus10Degrees) {
    constexpr double yaw_deg = 10.0;
    constexpr double yaw_rad = yaw_deg * kDegToRad;
    const Quaterniond q = eulerToQuaternion(0.0, 0.0, yaw_rad);

    // Vector Forward trong Body frame: [0, 1, 0]^T (hướng đỉnh máy)
    // Khi quay phải 10 độ (Yaw = +10 deg), trong Nav Frame (ENU):
    // v_nav phải có thành phần East > 0 (sin(10°)) và North = cos(10°)
    const Vector3d v_body{0.0, 1.0, 0.0};
    const Vector3d v_nav = rotateBodyToNav(q, v_body);

    EXPECT_NEAR(v_nav.x, std::sin(yaw_rad), 1e-4);
    EXPECT_NEAR(v_nav.y, std::cos(yaw_rad), 1e-4);
    EXPECT_NEAR(v_nav.z, 0.0, 1e-4);

    const double heading = extractCompassHeadingRad(q);
    EXPECT_NEAR(heading, yaw_rad, 1e-4);
}

TEST(QuaternionConventionsTest, PureYawMinus10Degrees) {
    constexpr double yaw_deg = -10.0;
    constexpr double yaw_rad = yaw_deg * kDegToRad;
    const Quaterniond q = eulerToQuaternion(0.0, 0.0, yaw_rad);

    // Khi quay trái 10 độ (Yaw = -10 deg / 350 deg):
    // v_nav phải có East < 0 (sin(-10°)) và North = cos(-10°)
    const Vector3d v_body{0.0, 1.0, 0.0};
    const Vector3d v_nav = rotateBodyToNav(q, v_body);

    EXPECT_NEAR(v_nav.x, std::sin(yaw_rad), 1e-4);
    EXPECT_NEAR(v_nav.y, std::cos(yaw_rad), 1e-4);
    EXPECT_NEAR(v_nav.z, 0.0, 1e-4);

    const double heading = extractCompassHeadingRad(q);
    EXPECT_NEAR(heading, kTwoPi + yaw_rad, 1e-4);
}

TEST(QuaternionConventionsTest, PurePitchPlus10Degrees) {
    constexpr double pitch_deg = 10.0;
    constexpr double pitch_rad = pitch_deg * kDegToRad;
    const Quaterniond q = eulerToQuaternion(0.0, pitch_rad, 0.0);

    // Vector Up trong Body frame: [0, 0, 1]^T (màn hình ngửa)
    // Nghiêng Pitch +10 độ quanh trục Y (Forward): Trục Z ngả sang trục X
    const Vector3d v_up_body{0.0, 0.0, 1.0};
    const Vector3d v_nav = rotateBodyToNav(q, v_up_body);

    EXPECT_NEAR(v_nav.z, std::cos(pitch_rad), 1e-4);
    EXPECT_NEAR(v_nav.x, std::sin(pitch_rad), 1e-4);
    EXPECT_NEAR(v_nav.y, 0.0, 1e-4);
}

TEST(QuaternionConventionsTest, PureRollPlus10Degrees) {
    constexpr double roll_deg = 10.0;
    constexpr double roll_rad = roll_deg * kDegToRad;
    const Quaterniond q = eulerToQuaternion(roll_rad, 0.0, 0.0);

    // Vector Up trong Body frame: [0, 0, 1]^T
    // Nghiêng Roll +10 độ quanh trục X (Right): Trục Z ngả sang trục Y (hoặc -Y)
    const Vector3d v_up_body{0.0, 0.0, 1.0};
    const Vector3d v_nav = rotateBodyToNav(q, v_up_body);

    EXPECT_NEAR(v_nav.z, std::cos(roll_rad), 1e-4);
    EXPECT_NEAR(v_nav.y, -std::sin(roll_rad), 1e-4);
    EXPECT_NEAR(v_nav.x, 0.0, 1e-4);
}

TEST(QuaternionConventionsTest, DcmOrthogonalityAndDeterminant) {
    const Quaterniond q = eulerToQuaternion(0.25, -0.15, 1.10);
    const Eigen::Matrix3d R = quaternionToRotationMatrix(q);

    // R * R^T = I
    const Eigen::Matrix3d I = R * R.transpose();
    EXPECT_TRUE(I.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    // det(R) = +1
    EXPECT_NEAR(R.determinant(), 1.0, 1e-6);
}
