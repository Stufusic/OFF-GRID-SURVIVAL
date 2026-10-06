#include "survival_core/kinematics/pdr_engine.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <cmath>

namespace survival::kinematics {

PdrEngine::PdrEngine(
    std::unique_ptr<IStepLengthModel> stride_model,
    const estimators::EskfConfig& eskf_config,
    const StepDetectorConfig& detector_config) noexcept
    : eskf_(eskf_config)
    , step_detector_(detector_config)
    , stride_model_(std::move(stride_model))
{
    if (!stride_model_) {
        stride_model_ = std::make_unique<AdaptiveParametricModel>();
    }
    reset();
}

void PdrEngine::reset(const EastingNorthingAlt& origin) noexcept {
    eskf_.reset();
    step_detector_.reset();
    current_position_ = origin;
    var_e_ = 0.01;
    var_n_ = 0.01;
    cov_en_ = 0.0;
    current_baro_alt_m_ = origin.altitude_m;
    last_step_alt_m_ = origin.altitude_m;
    has_initial_baro_ = false;
}

std::optional<StepEvent> PdrEngine::processFrame(const SensorFrame& frame, double dt_s) noexcept {
    // 1. Cập nhật ESKF 3D Fast Loop (100 Hz)
    eskf_.processSensorFrame(frame, dt_s);

    // 2. Cập nhật cao độ áp kế nếu có mẫu mới (lọc thông thấp khử nhiễu vi sai)
    if (frame.has_baro) {
        // P = P0 * (1 - h / 44330)^5.255 -> h = 44330 * (1 - (P / P0)^(1 / 5.255))
        const double p_ratio = static_cast<double>(frame.pressure_hpa) / kStandardPressureHpa;
        if (p_ratio > 0.0) {
            const double raw_alt = 44330.0 * (1.0 - std::pow(p_ratio, 1.0 / 5.255));
            if (!has_initial_baro_) {
                has_initial_baro_ = true;
                current_baro_alt_m_ = raw_alt;
                current_position_.altitude_m = current_baro_alt_m_;
                last_step_alt_m_ = current_baro_alt_m_;
            } else {
                current_baro_alt_m_ = 0.95 * current_baro_alt_m_ + 0.05 * raw_alt;
            }
        }
    }

    // 3. Trích xuất gia tốc thẳng đứng thực tế (đã xoay qua Quaternion và bù trọng lực)
    const Vector3f accel{frame.accel_m_s2[0], frame.accel_m_s2[1], frame.accel_m_s2[2]};
    const double a_v = eskf_.getVerticalAcceleration(accel);

    // 4. Phát hiện bước chân
    const bool is_step = step_detector_.processSample(a_v, frame.timestamp_ns);
    if (!is_step) {
        return std::nullopt;
    }

    // 5. Tính toán sải bước và tích phân vị trí
    StepInputFeatures features = step_detector_.getLastStepFeatures();
    const double delta_h = current_baro_alt_m_ - last_step_alt_m_;
    features.terrain_slope_rad = std::atan2(delta_h, 0.75); // Ước lượng góc dốc từ độ dịch chuyển cao độ

    const StrideEstimate stride = stride_model_->estimateStride(features);
    const double heading = eskf_.getHeadingRad();
    const double heading_var = eskf_.getAttitudeEstimate().heading_variance_rad2;

    const double sin_psi = std::sin(heading);
    const double cos_psi = std::cos(heading);
    const double L = stride.mean_length_m;
    const double var_L = stride.variance_m2;

    // Dịch chuyển vị trí 2D
    current_position_.easting_m += L * sin_psi;
    current_position_.northing_m += L * cos_psi;
    current_position_.altitude_m = current_baro_alt_m_;
    last_step_alt_m_ = current_baro_alt_m_;

    // Tích lũy ma trận hiệp phương sai vị trí P_pos
    // delta_P = [ var_L * sin^2 + L^2 * var_psi * cos^2,  (var_L - L^2 * var_psi) * sin * cos ]
    //           [ (var_L - L^2 * var_psi) * sin * cos  ,  var_L * cos^2 + L^2 * var_psi * sin^2 ]
    const double d_var_e = var_L * sin_psi * sin_psi + L * L * heading_var * cos_psi * cos_psi;
    const double d_var_n = var_L * cos_psi * cos_psi + L * L * heading_var * sin_psi * sin_psi;
    const double d_cov_en = (var_L - L * L * heading_var) * sin_psi * cos_psi;

    var_e_ += d_var_e;
    var_n_ += d_var_n;
    cov_en_ += d_cov_en;

    // Xuất sự kiện StepEvent
    StepEvent step{};
    step.timestamp_ns = frame.timestamp_ns;
    step.stride_mean_m = L;
    step.stride_variance_m2 = var_L;
    step.heading_rad = heading;
    step.heading_variance_rad2 = heading_var;
    step.vertical_displacement_m = delta_h;
    step.confidence = 1.0;

    return step;
}

} // namespace survival::kinematics
