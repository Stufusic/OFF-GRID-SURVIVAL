#pragma once

#include <cstdint>
#include <array>
#include "survival_core/types.hpp"

namespace survival::estimators {

struct EulerAnglesRad {
    // Frame BODY (docs/frame_conventions.md §II): +X = Phải, +Y = Trước, +Z = Lên.
    // Lưu ý: eulerToQuaternion() trong frame_conventions.hpp dùng tham số kiểu aerospace ZYX
    // (roll về trục X, pitch về trục Y) — KHÔNG dùng để tái tạo q từ 2 trường dưới đây.
    double roll_rad{0.0};    // [rad] Nghiêng NGANG (bank quanh trục trước +Y). Dương = nghiêng phải. [-PI, PI]
    double pitch_rad{0.0};   // [rad] Đổ VỀ TRƯỚC (quanh trục phải +X). Dương = trục trước chúc xuống. [-PI/2, PI/2]
    double yaw_rad{0.0};     // [rad] Góc phương vị Heading [0, 2*PI) (0 = North, East-positive)
};

struct AttitudeEstimate {
    uint64_t timestamp_ns{0};
    Quaterniond quaternion{1.0, 0.0, 0.0, 0.0};
    Vector3d gyro_bias_rad_s{0.0, 0.0, 0.0};
    EulerAnglesRad euler{};
    
    // Ma trận hiệp phương sai sai số góc 3x3 [P_roll, P_pitch, P_yaw]
    std::array<double, 9> attitude_cov_3x3{
        0.01, 0.0,  0.0,
        0.0,  0.01, 0.0,
        0.0,  0.0,  0.04
    };

    double heading_variance_rad2{0.04}; // Phương sai góc hướng yaw [rad^2]
    bool is_quasi_static{false};         // Cờ báo thiết bị đang ở trạng thái tĩnh
    bool is_mag_anomalous{false};       // Cờ báo từ trường bị nhiễu biến dạng
};

} // namespace survival::estimators
