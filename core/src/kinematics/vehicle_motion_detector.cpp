#include "survival_core/kinematics/vehicle_motion_detector.hpp"
#include <algorithm>

namespace survival::kinematics {

VehicleMotionDetector::VehicleMotionDetector() noexcept {
    reset();
}

void VehicleMotionDetector::reset() noexcept {
    high_band_history_.fill(0.0F);
    mid_band_history_.fill(0.0F);
    low_band_history_.fill(0.0F);
    window_idx_ = 0;
    sample_count_ = 0;
    lpf_low_state_ = 0.0F;
    lpf_mid_state_ = 0.0F;
    hpf_high_state_ = 0.0F;
    prev_accel_norm_ = 0.0F;
    stationary_time_s_ = 0.0F;
}

VehicleFeatures VehicleMotionDetector::processSample(const Vector3f& accel_m_s2, float gnss_speed_m_s, float human_cadence_hz, double dt_s) noexcept {
    VehicleFeatures feat{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        return feat;
    }

    const float ax = accel_m_s2.x;
    const float ay = accel_m_s2.y;
    const float az = accel_m_s2.z;
    const float a_norm = std::sqrt(ax * ax + ay * ay + az * az);
    const float dynamic_a = a_norm - static_cast<float>(kGravityStandard);

    // 1. Bộ lọc phân tích 3 dải phổ tần số IIR (ở 100 Hz, dt = 0.01)
    // Dải Low (fc ~ 3.5 Hz): alpha ~ 0.20
    lpf_low_state_ = 0.80F * lpf_low_state_ + 0.20F * dynamic_a;
    const float val_low = lpf_low_state_;

    // Dải Mid (fc ~ 12 Hz): alpha ~ 0.55
    lpf_mid_state_ = 0.45F * lpf_mid_state_ + 0.55F * (dynamic_a - val_low);
    const float val_mid = lpf_mid_state_;

    // Dải High (> 12 Hz): Phần dư tần số cao
    const float val_high = dynamic_a - val_low - val_mid;

    // Lưu vào bộ đệm cửa sổ trượt
    low_band_history_[window_idx_] = val_low * val_low;
    mid_band_history_[window_idx_] = val_mid * val_mid;
    high_band_history_[window_idx_] = val_high * val_high;
    window_idx_ = (window_idx_ + 1) % kWindowSize;
    if (sample_count_ < kWindowSize) {
        sample_count_++;
    }

    // 2. Tính toán năng lượng 3 dải (Mean Squared Energy)
    float sum_low = 0.0F;
    float sum_mid = 0.0F;
    float sum_high = 0.0F;
    const size_t n = (sample_count_ > 0) ? sample_count_ : 1;

    for (size_t i = 0; i < n; ++i) {
        sum_low += low_band_history_[i];
        sum_mid += mid_band_history_[i];
        sum_high += high_band_history_[i];
    }

    feat.energy_low_0_5_3_5hz = sum_low / static_cast<float>(n);
    feat.energy_mid_3_5_12hz = sum_mid / static_cast<float>(n);
    feat.energy_high_12_40hz = sum_high / static_cast<float>(n);

    // Tỷ số rung động cao tần
    feat.hf_ratio = feat.energy_high_12_40hz / (feat.energy_low_0_5_3_5hz + 1e-4F);

    // Entropy phổ 3 dải
    const float total_e = feat.energy_low_0_5_3_5hz + feat.energy_mid_3_5_12hz + feat.energy_high_12_40hz + 1e-6F;
    const float p1 = feat.energy_low_0_5_3_5hz / total_e;
    const float p2 = feat.energy_mid_3_5_12hz / total_e;
    const float p3 = feat.energy_high_12_40hz / total_e;

    float entropy = 0.0F;
    if (p1 > 1e-4F) entropy -= p1 * std::log2(p1);
    if (p2 > 1e-4F) entropy -= p2 * std::log2(p2);
    if (p3 > 1e-4F) entropy -= p3 * std::log2(p3);
    feat.spectral_entropy = entropy;

    // 3. Tính điểm Bằng chứng Phương tiện (Vehicle Evidence Score)
    float score = 0.0F;

    // Evidence A: GNSS Doppler Speed (nếu có tín hiệu GPS)
    if (gnss_speed_m_s >= 0.0F) {
        if (gnss_speed_m_s > 6.0F) { // > 21.6 km/h -> chắc chắn xe
            score += 0.85F;
        } else if (gnss_speed_m_s > 3.0F) { // 10.8 - 21.6 km/h -> khả năng cao xe/xe đạp
            score += 0.50F;
        }
    }

    // Evidence B: Rung động động cơ tần số cao kết hợp triệt tiêu bước chân
    const bool no_human_steps = (human_cadence_hz < 0.3F);
    if (no_human_steps) {
        if (feat.hf_ratio > 1.2F && feat.energy_high_12_40hz > 0.05F) {
            score += 0.75F;
        }
        if (feat.spectral_entropy > 0.8F) { // Phổ rung động cơ cơ giới rộng
            score += 0.15F;
        }
    } else {
        // Có bước chân người rõ ràng -> phạt điểm xe
        score = std::max(0.0F, score - 0.50F);
    }

    // Evidence C: Xe dừng đèn đỏ / đỗ xe (Không có bước chân người, GNSS speed ~ 0)
    if (no_human_steps && (gnss_speed_m_s >= 0.0F && gnss_speed_m_s < 0.5F)) {
        stationary_time_s_ += static_cast<float>(dt_s);
        if (stationary_time_s_ >= 1.0F) {
            feat.is_stopped = true;
            score = std::max(score, 0.70F); // Duy trì nhận diện xe ở trạng thái dừng
        }
    } else {
        stationary_time_s_ = 0.0F;
        feat.is_stopped = false;
    }

    feat.vehicle_score = std::clamp(score, 0.0F, 1.0F);
    return feat;
}

} // namespace survival::kinematics
