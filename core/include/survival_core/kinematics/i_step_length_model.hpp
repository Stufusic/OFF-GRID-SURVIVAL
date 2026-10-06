#pragma once

#include <cstdint>

namespace survival::kinematics {

struct StepInputFeatures {
    double cadence_hz{1.8};          // [Hz] Nhịp bước chân (steps per second)
    double vertical_accel_p2p{2.5};  // [m/s^2] Biến thiên gia tốc đỉnh - đáy thẳng đứng Delta a_v
    double vertical_accel_rms{0.8};  // [m/s^2] Giá trị RMS gia tốc thẳng đứng
    double terrain_slope_rad{0.0};   // [rad] Góc dốc địa hình (>0 leo dốc, <0 xuống dốc)
    double step_period_var{0.01};    // [s^2] Phương sai chu kỳ giữa các bước chân
    double user_leg_length_m{0.85};  // [m] Chiều dài chân người dùng
};

struct StrideEstimate {
    double mean_length_m{0.75};      // [m] Chiều dài bước kỳ vọng mu_L
    double variance_m2{0.0025};      // [m^2] Phương sai chiều dài bước sigma_L^2
};

class IStepLengthModel {
public:
    virtual ~IStepLengthModel() = default;

    /**
     * @brief Ước lượng chiều dài bước và phương sai từ các đặc trưng động học
     */
    [[nodiscard]] virtual StrideEstimate estimateStride(const StepInputFeatures& features) const noexcept = 0;
};

} // namespace survival::kinematics
