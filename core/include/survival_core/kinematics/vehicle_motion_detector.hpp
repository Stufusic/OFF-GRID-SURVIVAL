#pragma once

#include "survival_core/types.hpp"
#include <cstdint>
#include <array>
#include <cmath>

namespace survival::kinematics {

struct VehicleFeatures {
    float energy_low_0_5_3_5hz{0.0F};     // [m^2/s^4] Năng lượng dải nhịp bước người
    float energy_mid_3_5_12hz{0.0F};      // [m^2/s^4] Năng lượng dải chạy / địa hình gồ ghề
    float energy_high_12_40hz{0.0F};      // [m^2/s^4] Năng lượng dải rung động cơ xe
    float hf_ratio{0.0F};                 // Tỷ số E_high / E_low
    float spectral_entropy{0.0F};         // Entropy phổ
    float road_shock_density{0.0F};       // Mật độ va đập ổ gà / gờ giảm tốc
    float vehicle_score{0.0F};            // [0.0 - 1.0] Điểm bằng chứng phương tiện
    bool is_stopped{false};               // Xe đang dừng đỗ / dừng đèn đỏ
};

class VehicleMotionDetector {
public:
    VehicleMotionDetector() noexcept;

    /**
     * @brief Xử lý mẫu gia tốc (100 Hz) và vận tốc GNSS (nếu có)
     * @param accel_m_s2 Vector gia tốc thân máy
     * @param gnss_speed_m_s Vận tốc từ GNSS Doppler (hoặc âm nếu không có GNSS)
     * @param human_cadence_hz Nhịp bước người từ StepDetector
     * @param dt_s Bước thời gian [s]
     */
    VehicleFeatures processSample(const Vector3f& accel_m_s2, float gnss_speed_m_s, float human_cadence_hz, double dt_s) noexcept;

    void reset() noexcept;

private:
    // Cửa sổ trượt tĩnh 64 mẫu (0.64s ở 100Hz)
    static constexpr size_t kWindowSize = 64;
    std::array<float, kWindowSize> high_band_history_{};
    std::array<float, kWindowSize> mid_band_history_{};
    std::array<float, kWindowSize> low_band_history_{};
    size_t window_idx_{0};
    size_t sample_count_{0};

    // Bộ lọc IIR tách 3 dải tần số
    // Low: LPF 3.5 Hz
    // Mid: BPF 3.5 - 12 Hz
    // High: HPF 12 Hz
    float lpf_low_state_{0.0F};
    float lpf_mid_state_{0.0F};
    float hpf_high_state_{0.0F};
    float prev_accel_norm_{0.0F};

    // Theo dõi xe dừng
    float stationary_time_s_{0.0F};
};

} // namespace survival::kinematics
