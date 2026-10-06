#pragma once

#include <cstdint>
#include <array>
#include "survival_core/kinematics/bandpass_filter.hpp"
#include "survival_core/kinematics/i_step_length_model.hpp"

namespace survival::kinematics {

struct StepDetectorConfig {
    double min_step_interval_s{0.30};    // Sàn tuyệt đối giữa 2 bước chân [s] (max 3.3 Hz)
    double max_step_interval_s{1.20};    // Khoảng cách tối đa giữa 2 bước chân [s] (min 0.8 Hz)
    double min_peak_threshold_m_s2{0.05}; // Ngưỡng biên độ gia tốc tối thiểu để nhận diện bước
    // [A1] Sàn p2p đỉnh-đáy tuyệt đối: chặn bước ma (artifact gravity 1Hz MotionSense
    // khi đứng yên có p2p 0.05-0.2 m/s²). Bước chân thật ≥ 1.0 m/s² trên mọi dataset đo.
    double min_p2p_gate_m_s2{0.5};  // 0.5: ~2.5x bước ma; cân bằng đếm bước cầm tay biên thấp / drift
    double k_dynamic_threshold{0.4};     // Hệ số ngưỡng động Th = mu + k * sigma
};

class StepDetector {
public:
    explicit StepDetector(const StepDetectorConfig& config = StepDetectorConfig{}) noexcept;

    /**
     * @brief Xử lý 1 mẫu gia tốc thẳng đứng a_v (100 Hz)
     * @param vertical_accel_m_s2 Gia tốc thẳng đứng đã bù trọng lực (a_nav,z - g)
     * @param timestamp_ns Nhãn thời gian nano-giây
     * @return true nếu phát hiện đỉnh bước chân hợp lệ tại mẫu này
     */
    [[nodiscard]] bool processSample(double vertical_accel_m_s2, uint64_t timestamp_ns) noexcept;

    /**
     * @brief Lấy các đặc trưng động học trích xuất từ bước chân vừa phát hiện
     */
    [[nodiscard]] const StepInputFeatures& getLastStepFeatures() const noexcept {
        return last_features_;
    }

    [[nodiscard]] uint64_t getLastStepTimestampNs() const noexcept {
        return last_step_timestamp_ns_;
    }

    [[nodiscard]] size_t getTotalStepCount() const noexcept {
        return total_step_count_;
    }

    void reset() noexcept;

private:
    StepDetectorConfig config_{};
    BandpassFilter filter_{};

    // Bộ đệm tĩnh 3 mẫu để phát hiện cực trị (Local Maxima / Minima)
    std::array<double, 3> sample_window_{0.0, 0.0, 0.0};
    
    // Cửa sổ trượt tĩnh 64 mẫu để tính RMS và năng lượng
    static constexpr size_t kHistoryCapacity = 64;
    std::array<double, kHistoryCapacity> energy_history_{};
    size_t history_idx_{0};
    size_t history_count_{0};

    // Lịch sử 4 chu kỳ bước chân gần nhất để tính phương sai chu kỳ
    std::array<double, 4> recent_periods_s_{0.55, 0.55, 0.55, 0.55};
    size_t period_idx_{0};

    uint64_t last_step_timestamp_ns_{0};
    uint64_t last_sample_ts_ns_{0};  // [C1] mốc đo dt để thích ứng Fs cho bandpass
    double current_peak_val_{0.0};
    double current_valley_val_{0.0};
    bool is_searching_peak_{true};
    size_t total_step_count_{0};

    StepInputFeatures last_features_{};

    void updateHistory(double filtered_val) noexcept;
    [[nodiscard]] double computeRms() const noexcept;
    [[nodiscard]] double computePeriodVariance() const noexcept;
};

} // namespace survival::kinematics
