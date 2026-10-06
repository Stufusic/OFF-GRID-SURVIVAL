#pragma once

#include <cstdint>
#include <array>
#include "survival_core/types.hpp"

namespace survival::safety {

struct SafetyConfig {
    double fall_impact_g{3.5};                 // Ngưỡng va chạm ngã (G)
    double tumble_gyro_rad_s{3.0};             // Ngưỡng xoay cơ thể mất kiểm soát (rad/s)
    uint64_t tumble_window_ns{300000000ULL};   // Cửa sổ kiểm tra xoay trước va chạm (300 ms)
    uint64_t pre_alert_duration_ns{45000000000ULL}; // Thời gian đếm ngược hủy cảnh báo (45 s)
    uint64_t immobility_threshold_ns{180000000000ULL}; // Ngưỡng bất động sau ngã (3 phút = 180s)
    double immobility_accel_noise_m_s2{0.20};  // Độ dao động gia tốc khi bất động
    size_t panic_step_min{500};                // Số bước tối thiểu để xét đi vòng quanh
    double panic_displacement_ratio{0.10};     // Tỷ lệ D_net / S_total
};

class SafetyAuditor {
public:
    explicit SafetyAuditor(const SafetyConfig& config = SafetyConfig{}) noexcept;

    /**
     * @brief Cập nhật mẫu cảm biến chuyển động IMU (100 Hz)
     */
    void processMotionSample(const Vector3f& accel_m_s2, const Vector3f& gyro_rad_s, uint64_t timestamp_ns) noexcept;

    /**
     * @brief Cập nhật tiến trình bước chân và độ dịch chuyển quỹ đạo PDR
     */
    void updateOdometryDisplacement(double net_displacement_m, double total_path_distance_m, size_t total_steps) noexcept;

    /**
     * @brief Người dùng chạm màn hình hoặc bấm nút hủy cảnh báo
     */
    void cancelAlert() noexcept;

    /**
     * @brief Kích hoạt cứu nạn thủ công (SOS Button Pressed)
     */
    void triggerManualSos() noexcept;

    [[nodiscard]] SafetyState getState() const noexcept {
        return state_;
    }

    [[nodiscard]] double getPreAlertRemainingSeconds() const noexcept;

    [[nodiscard]] bool isDistressActive() const noexcept {
        return state_ == SafetyState::kDistressLocked;
    }

    [[nodiscard]] bool isDeviceDropOnly() const noexcept {
        return is_last_drop_device_only_;
    }

    [[nodiscard]] bool isPanicCirclingWarning() const noexcept {
        return is_panic_circling_;
    }

    void reset() noexcept;

private:
    SafetyConfig config_{};
    SafetyState state_{SafetyState::kNormal};

    // Bộ đệm trượt 32 mẫu gyro (320ms ở 100Hz) để bắt chuỗi xoay tròn trước va chạm (Tumble Dynamics)
    static constexpr size_t kGyroHistoryCapacity = 32;
    std::array<double, kGyroHistoryCapacity> gyro_norm_history_{};
    std::array<uint64_t, kGyroHistoryCapacity> gyro_time_history_{};
    size_t gyro_idx_{0};
    size_t gyro_count_{0};

    uint64_t pre_alert_start_ns_{0};
    uint64_t last_fall_impact_ns_{0};
    uint64_t last_motion_ns_{0};
    uint64_t current_timestamp_ns_{0};

    bool is_last_drop_device_only_{false};
    bool is_panic_circling_{false};
};

} // namespace survival::safety
