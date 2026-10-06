#include "survival_core/safety/safety_auditor.hpp"
#include <cmath>
#include <algorithm>

namespace survival::safety {

SafetyAuditor::SafetyAuditor(const SafetyConfig& config) noexcept
    : config_(config)
{
}

void SafetyAuditor::reset() noexcept {
    state_ = SafetyState::kNormal;
    gyro_idx_ = 0;
    gyro_count_ = 0;
    pre_alert_start_ns_ = 0;
    last_fall_impact_ns_ = 0;
    last_motion_ns_ = 0;
    current_timestamp_ns_ = 0;
    is_last_drop_device_only_ = false;
    is_panic_circling_ = false;
}

void SafetyAuditor::processMotionSample(const Vector3f& accel_m_s2, const Vector3f& gyro_rad_s, uint64_t timestamp_ns) noexcept {
    current_timestamp_ns_ = timestamp_ns;

    const double ax = accel_m_s2.x;
    const double ay = accel_m_s2.y;
    const double az = accel_m_s2.z;
    const double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
    const double accel_g = accel_norm / kGravityStandard;

    const double gx = gyro_rad_s.x;
    const double gy = gyro_rad_s.y;
    const double gz = gyro_rad_s.z;
    const double gyro_norm = std::sqrt(gx * gx + gy * gy + gz * gz);

    // 1. Kiểm tra chuyển động động học (Dynamic Motion) — cập nhật TRƯỚC máy trạng
    // thái để escalation bất động [M8b] dùng giá trị mới nhất của mẫu hiện tại
    const double dynamic_accel = std::abs(accel_norm - kGravityStandard);
    if (dynamic_accel > config_.immobility_accel_noise_m_s2 || gyro_norm > 0.3) {
        last_motion_ns_ = timestamp_ns;
    }

    // 2. Xử lý máy trạng thái — [M8a] chạy TRƯỚC khi push mẫu hiện tại vào lịch sử,
    // để cửa sổ tumble 300ms chỉ quét các mẫu TRƯỚC va chạm (trước đây mẫu impact
    // tự nó rơi vào cửa sổ khiến rơi MÁY đơn thuần bị nhận nhầm ngã người)
    if (state_ == SafetyState::kNormal) {
        if (accel_g >= config_.fall_impact_g) {
            // Kiểm tra xem có xoay tròn mất kiểm soát trong 300ms trước va chạm không
            bool has_prior_tumble = false;
            for (size_t i = 0; i < gyro_count_; ++i) {
                const uint64_t t_sample = gyro_time_history_[i];
                if (timestamp_ns >= t_sample && (timestamp_ns - t_sample) <= config_.tumble_window_ns) {
                    if (gyro_norm_history_[i] >= config_.tumble_gyro_rad_s) {
                        has_prior_tumble = true;
                        break;
                    }
                }
            }

            if (has_prior_tumble) {
                // Cú ngã thực tế của con người -> Chuyển PRE_ALERT đếm ngược 45s
                state_ = SafetyState::kPreAlert;
                pre_alert_start_ns_ = timestamp_ns;
                last_fall_impact_ns_ = timestamp_ns;
                is_last_drop_device_only_ = false;
            } else {
                // Rơi thiết bị đơn thuần không có xoay cơ thể
                is_last_drop_device_only_ = true;
            }
        }
    } else if (state_ == SafetyState::kPreAlert) {
        // Hết thời gian đếm ngược 45s -> Khóa trạng thái khẩn cấp
        if (timestamp_ns >= pre_alert_start_ns_ + config_.pre_alert_duration_ns) {
            state_ = SafetyState::kDistressLocked;
        }

        // [M8b] Bất động: cửa sổ trượt tính từ LẦN CỬ ĐỘNG CUỐI (độc lập countdown).
        // Trước đây so với mốc ngã + điều kiện 'last_motion <= ngã + 5s' khiến nhánh
        // này là dead code (45s < 180s) và bỏ sót nạn nhân cử động lại rồi nằm yên.
        if (last_motion_ns_ > 0 && timestamp_ns > last_motion_ns_ &&
            (timestamp_ns - last_motion_ns_) >= config_.immobility_threshold_ns) {
            state_ = SafetyState::kDistressLocked;
        }
    }

    // 3. Push mẫu hiện tại vào lịch sử tốc độ góc (SAU máy trạng thái — M8a)
    gyro_norm_history_[gyro_idx_] = gyro_norm;
    gyro_time_history_[gyro_idx_] = timestamp_ns;
    gyro_idx_ = (gyro_idx_ + 1) % kGyroHistoryCapacity;
    if (gyro_count_ < kGyroHistoryCapacity) {
        gyro_count_++;
    }
}

void SafetyAuditor::updateOdometryDisplacement(double net_displacement_m, double total_path_distance_m, size_t total_steps) noexcept {
    if (total_steps >= config_.panic_step_min && total_path_distance_m > 100.0) {
        const double ratio = net_displacement_m / total_path_distance_m;
        if (ratio < config_.panic_displacement_ratio) {
            is_panic_circling_ = true;
            if (state_ == SafetyState::kNormal) {
                state_ = SafetyState::kPanicCirclingWarning;
            }
            return;
        }
    }
    is_panic_circling_ = false;
    if (state_ == SafetyState::kPanicCirclingWarning) {
        state_ = SafetyState::kNormal;
    }
}

void SafetyAuditor::cancelAlert() noexcept {
    if (state_ == SafetyState::kPreAlert || state_ == SafetyState::kDistressLocked || state_ == SafetyState::kPanicCirclingWarning) {
        state_ = SafetyState::kNormal;
        pre_alert_start_ns_ = 0;
        last_fall_impact_ns_ = 0;
    }
}

void SafetyAuditor::triggerManualSos() noexcept {
    state_ = SafetyState::kDistressLocked;
}

double SafetyAuditor::getPreAlertRemainingSeconds() const noexcept {
    if (state_ != SafetyState::kPreAlert || current_timestamp_ns_ < pre_alert_start_ns_) {
        return 0.0;
    }
    const uint64_t elapsed_ns = current_timestamp_ns_ - pre_alert_start_ns_;
    if (elapsed_ns >= config_.pre_alert_duration_ns) {
        return 0.0;
    }
    return static_cast<double>(config_.pre_alert_duration_ns - elapsed_ns) * 1e-9;
}

} // namespace survival::safety
