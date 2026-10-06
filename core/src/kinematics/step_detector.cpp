#include "survival_core/kinematics/step_detector.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace survival::kinematics {

StepDetector::StepDetector(const StepDetectorConfig& config) noexcept
    : config_(config)
{
    reset();
}

void StepDetector::reset() noexcept {
    filter_.reset();
    last_sample_ts_ns_ = 0;
    sample_window_ = {0.0, 0.0, 0.0};
    energy_history_.fill(0.0);
    history_idx_ = 0;
    history_count_ = 0;
    recent_periods_s_ = {0.55, 0.55, 0.55, 0.55};
    period_idx_ = 0;
    last_step_timestamp_ns_ = 0;
    current_peak_val_ = 0.0;
    current_valley_val_ = -0.5;
    is_searching_peak_ = true;
    total_step_count_ = 0;
    last_features_ = StepInputFeatures{};
    last_features_.cadence_hz = 0.0;
}

void StepDetector::updateHistory(double filtered_val) noexcept {
    energy_history_[history_idx_] = filtered_val;
    history_idx_ = (history_idx_ + 1) % kHistoryCapacity;
    if (history_count_ < kHistoryCapacity) {
        history_count_++;
    }
}

double StepDetector::computeRms() const noexcept {
    if (history_count_ == 0) {
        return 0.0;
    }
    double sum_sq = 0.0;
    for (size_t i = 0; i < history_count_; ++i) {
        sum_sq += energy_history_[i] * energy_history_[i];
    }
    return std::sqrt(sum_sq / static_cast<double>(history_count_));
}

double StepDetector::computePeriodVariance() const noexcept {
    double sum = 0.0;
    for (double p : recent_periods_s_) {
        sum += p;
    }
    const double mean = sum / 4.0;
    double var = 0.0;
    for (double p : recent_periods_s_) {
        const double diff = p - mean;
        var += diff * diff;
    }
    return var / 3.0; // Sample variance
}

bool StepDetector::processSample(double vertical_accel_m_s2, uint64_t timestamp_ns) noexcept {
    // [C1] Thích ứng tần số lấy mẫu theo dt quan sát được (50/100/200 Hz IMU):
    // hệ số lọc cứng cho 100 Hz sẽ méo dải thông ở tốc độ lấy mẫu khác.
    if (last_sample_ts_ns_ != 0 && timestamp_ns > last_sample_ts_ns_) {
        const double dt = static_cast<double>(timestamp_ns - last_sample_ts_ns_) * 1e-9;
        if (dt > 0.0005 && dt < 0.1) {
            filter_.setSampleRate(1.0 / dt);
        }
    }
    last_sample_ts_ns_ = timestamp_ns;

    // 1. Lọc thông dải IIR
    const double filtered = filter_.process(vertical_accel_m_s2);
    updateHistory(filtered);

    // 2. Cập nhật cửa sổ 3 mẫu tìm cực trị
    sample_window_[0] = sample_window_[1];
    sample_window_[1] = sample_window_[2];
    sample_window_[2] = filtered;

    const double w0 = sample_window_[0];
    const double w1 = sample_window_[1]; // Điểm giữa xét cực trị
    const double w2 = sample_window_[2];

    // Kiểm tra đáy (Local Valley)
    if (w1 < w0 && w1 < w2) {
        current_valley_val_ = w1;
    }

    // Kiểm tra đỉnh (Local Peak)
    if (w1 > w0 && w1 > w2) {
        const double dynamic_threshold = std::max(config_.min_peak_threshold_m_s2, computeRms() * config_.k_dynamic_threshold);

        if (w1 >= dynamic_threshold) {
            current_peak_val_ = w1;

            // [A1] Cổng p2p tuyệt đối: bước ma (artifact gravity 1Hz của MotionSense
            // khi đứng yên) có biên độ 0.05-0.2 m/s² — bước chân thật luôn ≥ 1.0.
            const double step_p2p = std::abs(current_peak_val_ - current_valley_val_);
            if (step_p2p < config_.min_p2p_gate_m_s2) {
                return false;
            }

            if (last_step_timestamp_ns_ == 0) {
                // Bước đầu tiên: yêu cầu độ chênh đỉnh-đáy rõ ràng của bước chân người
                const double p2p = std::abs(current_peak_val_ - current_valley_val_);
                if (p2p >= 2.0) {
                    last_step_timestamp_ns_ = timestamp_ns;
                    total_step_count_++;
                    last_features_.cadence_hz = 1.8;
                    last_features_.vertical_accel_p2p = p2p;
                    last_features_.vertical_accel_rms = computeRms();
                    last_features_.step_period_var = 0.01;
                    return true;
                }
                return false;
            }

            const double dt_step_s = static_cast<double>(timestamp_ns - last_step_timestamp_ns_) * 1e-9;

            // [C1] Luật tỉ lệ chống double-peak: khi ESKF tilt đúng, harmonics của
            // sải (đo D2: 28/121 khoảng 0.35-0.40s = 0.53-0.60 chu kỳ thật) tạo
            // đỉnh giả. Double-peak luôn < 0.65×chu kỳ trung bình; bước thật ≥ 0.9×.
            const double median_period = [&] {
                double p[4] = {recent_periods_s_[0], recent_periods_s_[1],
                               recent_periods_s_[2], recent_periods_s_[3]};
                std::sort(p, p + 4);
                return 0.5 * (p[1] + p[2]);
            }();
            // Trần 0.45s: landing/đứng nghỉ giữa các vế kéo median chu kỳ lên
            // (0.8-1.2s) — không trần thì các bước thang nhanh 0.55s bị nuốt (D1).
            const double effective_min = std::min(
                std::max(config_.min_step_interval_s, 0.65 * median_period), 0.45);

            if (dt_step_s >= effective_min && dt_step_s <= config_.max_step_interval_s) {
                last_step_timestamp_ns_ = timestamp_ns;
                total_step_count_++;

                recent_periods_s_[period_idx_] = dt_step_s;
                period_idx_ = (period_idx_ + 1) % 4;

                last_features_.cadence_hz = 1.0 / dt_step_s;
                last_features_.vertical_accel_p2p = std::abs(current_peak_val_ - current_valley_val_);
                last_features_.vertical_accel_rms = computeRms();
                last_features_.step_period_var = computePeriodVariance();

                return true;
            } else if (dt_step_s > config_.max_step_interval_s) {
                // Người dùng dừng lại một lúc rồi đi tiếp -> reset nhịp.
                // [A1] FIX: nhánh cũ chấp nhận MỌI đỉnh sau nghỉ (không gate) —
                // bước ma khi đứng yên được đếm như bước chân thật. Gate 0.8 m/s²:
                // 4x biên độ bước ma (0.05-0.2) nhưng không nuốt bước đi thật sau
                // nghỉ trên dataset cầm tay biên độ thấp (2.0 làm D2 mất 37% bước).
                if (step_p2p < 0.8) {
                    return false;
                }
                last_step_timestamp_ns_ = timestamp_ns;
                total_step_count_++;
                last_features_.cadence_hz = 1.0 / std::max(0.30, dt_step_s);
                last_features_.vertical_accel_p2p = step_p2p;
                last_features_.vertical_accel_rms = computeRms();
                last_features_.step_period_var = 0.01;
                return true;
            }
        }
    }

    return false;
}

} // namespace survival::kinematics
