#include <gtest/gtest.h>
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

using namespace survival;
using namespace survival::estimators;
using namespace survival::math;

TEST(Eskf3DTest, StaticConvergenceToNorth) {
    Eskf3D eskf;
    // Khởi tạo lệch nhẹ
    eskf.reset(Quaterniond{1.0, 0.0, 0.0, 0.0});

    // Mẫu cảm biến tĩnh: Gia tốc [0, 0, g], Từ trường hướng Bắc [0, 30, -40] uT
    const Vector3f accel{0.0F, 0.0F, static_cast<float>(kGravityStandard)};
    const Vector3f gyro{0.0F, 0.0F, 0.0F};
    const Vector3f mag{0.0F, 30.0F, -40.0F};

    constexpr double dt = 0.01; // 100 Hz
    for (int i = 0; i < 100; ++i) {
        eskf.predict(gyro, dt);
        eskf.updateAccelerometer(accel);
        eskf.updateMagnetometer(mag);
    }

    const auto att = eskf.getAttitudeEstimate();
    EXPECT_NEAR(att.euler.roll_rad, 0.0, 0.02);
    EXPECT_NEAR(att.euler.pitch_rad, 0.0, 0.02);
    EXPECT_NEAR(att.euler.yaw_rad, 0.0, 0.02); // Hướng Bắc
    EXPECT_TRUE(att.is_quasi_static);
    EXPECT_FALSE(att.is_mag_anomalous);
}

TEST(Eskf3DTest, PureZRotation90Degrees) {
    Eskf3D eskf;
    eskf.reset();

    // Quay phải 90 độ (North -> East) với vận tốc góc -0.314159 rad/s trong 5 giây
    constexpr double rate = kHalfPi / 5.0; // 0.314159 rad/s
    const Vector3f gyro{0.0F, 0.0F, static_cast<float>(-rate)};
    constexpr double dt = 0.01; // 100 Hz
    constexpr int steps = 500;  // 5 giây

    for (int i = 0; i < steps; ++i) {
        eskf.predict(gyro, dt);
    }

    const double heading = eskf.getHeadingRad();
    EXPECT_NEAR(heading, kHalfPi, 0.01); // 90 độ (Hướng Đông)
}

TEST(Eskf3DTest, QuasiStaticGravityGateRejection) {
    Eskf3D eskf;
    eskf.reset();

    // Giả lập cú sốc gia tốc động học mạnh: a = 25 m/s^2 (không phải trọng lực tĩnh)
    const Vector3f dynamic_accel{10.0F, 10.0F, 20.0F};
    const bool accepted = eskf.updateAccelerometer(dynamic_accel);

    EXPECT_FALSE(accepted); // Bắt buộc bị Gate từ chối
    EXPECT_FALSE(eskf.getAttitudeEstimate().is_quasi_static);
}

TEST(Eskf3DTest, MagneticAnomalyRejection) {
    Eskf3D eskf;
    eskf.reset();

    // Nhiễu từ trường cực mạnh 150 uT (gần dây điện cao thế / kim loại lớn)
    const Vector3f spike_mag{120.0F, 80.0F, 50.0F}; // norm = 152 uT
    const bool accepted = eskf.updateMagnetometer(spike_mag);

    EXPECT_FALSE(accepted); // Bắt buộc bị loại bỏ
    EXPECT_TRUE(eskf.getAttitudeEstimate().is_mag_anomalous);
}

TEST(Eskf3DTest, VerticalAccelerationExtractionWithPitch) {
    Eskf3D eskf;
    // Khởi tạo máy nghiêng Pitch 45 độ (quay quanh trục Y góc PI/4)
    const double half_angle = kPi / 8.0;
    const Quaterniond q_pitch45{std::cos(half_angle), 0.0, std::sin(half_angle), 0.0};
    eskf.reset(q_pitch45);

    // Trong Body Frame khi máy nghiêng 45 độ và có thêm gia tốc giật nảy thẳng đứng 2.0 m/s^2:
    // a_nav = [0, 0, g + 2.0]^T -> a_body = R^T * [0, 0, g + 2.0]^T
    const double g_plus_2 = kGravityStandard + 2.0;
    const Vector3f a_body{
        static_cast<float>(-g_plus_2 * std::sin(kPi / 4.0)),
        0.0F,
        static_cast<float>(g_plus_2 * std::cos(kPi / 4.0))
    };

    const double a_v = eskf.getVerticalAcceleration(a_body);
    // Gia tốc thẳng đứng thực tế phải xấp xỉ +2.0 m/s^2 (bất chấp máy bị nghiêng 45 độ)
    EXPECT_NEAR(a_v, 2.0, 0.05);
}

// [H1] Ngữ nghĩa euler vật lý: pitch dương = đổ về trước (trục trước chúc xuống)
TEST(Eskf3DTest, ForwardLeanIsPositivePitch) {
    Eskf3D eskf;
    // Đổ về trước 30 độ = xoay quanh trục phải (+X) chiều âm 30° (quaternion x = sin(-15°))
    const double half = (30.0 * kDegToRad) / 2.0;
    const Quaterniond q_fwd_lean{std::cos(half), -std::sin(half), 0.0, 0.0};
    eskf.reset(q_fwd_lean);

    const auto att = eskf.getAttitudeEstimate();
    EXPECT_NEAR(att.euler.pitch_rad, 30.0 * kDegToRad, 1e-6); // dương = chúc xuống
    EXPECT_NEAR(att.euler.roll_rad, 0.0, 1e-6);
    EXPECT_NEAR(att.euler.yaw_rad, 0.0, 1e-6);

    // Ngửa ra sau 25 độ phải cho pitch âm
    const double half2 = (25.0 * kDegToRad) / 2.0;
    const Quaterniond q_lean_back{std::cos(half2), std::sin(half2), 0.0, 0.0};
    eskf.reset(q_lean_back);
    EXPECT_NEAR(eskf.getAttitudeEstimate().euler.pitch_rad, -25.0 * kDegToRad, 1e-6);
}

// [H1] Ngữ nghĩa euler vật lý: roll dương = nghiêng phải (bank quanh trục trước +Y)
TEST(Eskf3DTest, RightBankIsPositiveRoll) {
    Eskf3D eskf;
    // Nghiêng phải 20 độ = xoay quanh trục trước (+Y) góc +20°
    const double half = (20.0 * kDegToRad) / 2.0;
    const Quaterniond q_bank_right{std::cos(half), 0.0, std::sin(half), 0.0};
    eskf.reset(q_bank_right);

    const auto att = eskf.getAttitudeEstimate();
    EXPECT_NEAR(att.euler.roll_rad, 20.0 * kDegToRad, 1e-6); // dương = nghiêng phải
    EXPECT_NEAR(att.euler.pitch_rad, 0.0, 1e-6);
    EXPECT_NEAR(att.euler.yaw_rad, 0.0, 1e-6);

    // Nghiêng trái 15 độ phải cho roll âm
    const double half2 = (15.0 * kDegToRad) / 2.0;
    const Quaterniond q_bank_left{std::cos(half2), 0.0, -std::sin(half2), 0.0};
    eskf.reset(q_bank_left);
    EXPECT_NEAR(eskf.getAttitudeEstimate().euler.roll_rad, -15.0 * kDegToRad, 1e-6);
}


// [B3] Tu sai phai duoc tru khoi innovation — khong config: heading hoi tu ve
// Bac TU (lech ca D); co config D=10 do: heading hoi tu ve 0 dung
TEST(Eskf3DTest, MagneticDeclinationCompensated) {
    const double d = 10.0 * kDegToRad;
    const float mx = static_cast<float>(30.0 * std::sin(d));
    const float my = static_cast<float>(30.0 * std::cos(d));

    // --- KHONG config declination: heading hoi tu ve +10 do (sai) ---
    Eskf3D eskf_no_comp;
    for (int i = 0; i < 200; ++i) {
        eskf_no_comp.predict(Vector3f{0.0F, 0.0F, 0.0F}, 0.01);
        eskf_no_comp.updateAccelerometer(Vector3f{0.0F, 0.0F, static_cast<float>(kGravityStandard)});
        eskf_no_comp.updateMagnetometer(Vector3f{mx, my, -40.0F});
    }
    // Nghiem chung: khong buu -> heading = -D (Bac tu); getHeadingRad tra [0, 2pi)
    const double no_comp_head = eskf_no_comp.getHeadingRad();
    EXPECT_NEAR(std::atan2(std::sin(no_comp_head + d), std::cos(no_comp_head + d)), 0.0, 0.03);

    // --- Co config declination 10 do: heading hoi tu ve 0 (dung) ---
    EskfConfig cfg{};
    cfg.mag_declination_rad = d;
    Eskf3D eskf_comp{cfg};
    for (int i = 0; i < 200; ++i) {
        eskf_comp.predict(Vector3f{0.0F, 0.0F, 0.0F}, 0.01);
        eskf_comp.updateAccelerometer(Vector3f{0.0F, 0.0F, static_cast<float>(kGravityStandard)});
        eskf_comp.updateMagnetometer(Vector3f{mx, my, -40.0F});
    }
    const double comp_head = eskf_comp.getHeadingRad();
    EXPECT_NEAR(std::atan2(std::sin(comp_head), std::cos(comp_head)), 0.0, 0.03);  // wrap [0,2pi)
}
