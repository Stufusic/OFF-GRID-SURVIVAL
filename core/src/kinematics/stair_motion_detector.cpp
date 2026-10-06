#include "survival_core/kinematics/stair_motion_detector.hpp"
#include <algorithm>
#include <cmath>

namespace survival::kinematics {

StairMotionDetector::StairMotionDetector(const StairMotionDetectorConfig& config) noexcept
    : config_(config)
{
    reset();
}

void StairMotionDetector::reset() noexcept {
    current_trough_val_ = 0.0;
    current_peak_val_ = 0.0;
    time_since_trough_s_ = 0.0;
    time_since_peak_s_ = 0.0;
    min_during_step_m_s2_ = 999.0;
    freefall_time_s_ = 0.0;
    is_in_rising_phase_ = false;

    current_v_z_m_s_ = 0.0;
    step_displacement_z_m_ = 0.0;

    last_ascent_score_ = 0.0F;
    last_descent_score_ = 0.0F;
    last_vertical_jerk_m_s3_ = 0.0F;
    last_skewness_ = 0.0F;
    last_step_delta_z_ = 0.0F;
    last_zupt_confidence_ = 0.0F;
    last_freefall_ms_ = 0.0F;
    last_is_landing_ = false;
    time_since_last_step_s_ = 0.0F;
    consecutive_low_p_stair_steps_ = 0;
    flight_p2p_median_ema_ = 10.0F;
    consecutive_landing_steps_ = 0;

    step_buf_len_ = 0;
    step_accel_buf_.fill(0.0F);

    window_idx_ = 0;
    window_filled_ = false;
    window_vert_accel_.fill(0.0F);
    window_horiz_accel_.fill(0.0F);

    recent_p2p_.fill(0.0F);
    history_idx_ = 0;

    state_ = StairMotionState{};
    consecutive_ascent_steps_ = 0;
    consecutive_descent_steps_ = 0;
}

float StairMotionDetector::computeSkewness(const float* data, size_t len) noexcept {
    if (data == nullptr || len < 6) {
        return 0.0F;
    }

    // Tính Temporal Energy Asymmetry: Năng lượng nửa đầu vs Nửa cuối chu kỳ bước
    // Lên thang: Lực đẩy (Push-off) dồn ở nửa đầu -> Asymmetry > +0.10
    // Xuống thang: Va đập (Impact spike) dồn ở nửa cuối -> Asymmetry < -0.10
    const size_t half = len / 2;
    double e_first = 0.0;
    for (size_t i = 0; i < half; ++i) {
        e_first += static_cast<double>(data[i] * data[i]);
    }
    double e_second = 0.0;
    for (size_t i = half; i < len; ++i) {
        e_second += static_cast<double>(data[i] * data[i]);
    }

    const double total_energy = e_first + e_second + 1e-6;
    const double asymmetry = (e_first - e_second) / total_energy;
    return static_cast<float>(std::clamp(asymmetry, -1.0, 1.0));
}

StairDetectorFeatures StairMotionDetector::processSample(
    double vertical_accel_m_s2,
    double horizontal_accel_norm,
    double dt_s,
    bool is_step_peak,
    float orientation_confidence
) noexcept {
    StairDetectorFeatures feat{};

    if (dt_s <= 0.0 || dt_s > 0.5) {
        return feat;
    }

    time_since_trough_s_ += dt_s;
    time_since_peak_s_ += dt_s;
    time_since_last_step_s_ += static_cast<float>(dt_s);

    // 1. Cập nhật cửa sổ phương sai
    window_vert_accel_[window_idx_] = static_cast<float>(vertical_accel_m_s2);
    window_horiz_accel_[window_idx_] = static_cast<float>(horizontal_accel_norm);
    window_idx_ = (window_idx_ + 1) % kVarianceWindow;
    if (window_idx_ == 0) {
        window_filled_ = true;
    }

    // 2. Lưu mẫu vào buffer chu kỳ bước
    if (step_buf_len_ < kMaxStepBuffer) {
        step_accel_buf_[step_buf_len_++] = static_cast<float>(vertical_accel_m_s2);
    }

    // 3. Đo thời gian hụt trọng lực
    if (vertical_accel_m_s2 < -2.0) {
        freefall_time_s_ += dt_s;
    }

    // 4. Tích phân số vận tốc thẳng đứng từng bước
    current_v_z_m_s_ += vertical_accel_m_s2 * dt_s;
    current_v_z_m_s_ *= 0.995; // Bù trôi nhẹ
    step_displacement_z_m_ += current_v_z_m_s_ * dt_s;

    if (vertical_accel_m_s2 < min_during_step_m_s2_) {
        min_during_step_m_s2_ = vertical_accel_m_s2;
    }

    if (vertical_accel_m_s2 < current_trough_val_ || !is_in_rising_phase_) {
        if (vertical_accel_m_s2 < -0.5) {
            current_trough_val_ = vertical_accel_m_s2;
            time_since_trough_s_ = 0.0;
            is_in_rising_phase_ = true;
        }
    }

    // Timeout dừng chân
    if (time_since_last_step_s_ > 1.8F) {
        if (time_since_last_step_s_ <= 3.5F) {
            if (state_.activity == StairActivity::kAscent || state_.activity == StairActivity::kDescent) {
                state_.activity = StairActivity::kPaused;
            }
        } else if (time_since_last_step_s_ <= 5.5F) {
            state_.flight_confidence *= 0.95F;
            state_.p_stair *= 0.95F;
        } else {
            state_.activity = StairActivity::kNone;
            state_.phase = FlightPhase::kUnknown;
            state_.flight_step_count = 0;
            consecutive_ascent_steps_ = 0;
            consecutive_descent_steps_ = 0;
            last_ascent_score_ = 0.0F;
            last_descent_score_ = 0.0F;
        }
    }

    // 5. Khi phát hiện đỉnh bước chân
    if (is_step_peak) {
        const float step_interval_s = time_since_last_step_s_; // khoảng bước TRƯỚC khi reset
        time_since_last_step_s_ = 0.0F;

        // Trích xuất cực trị trực tiếp từ buffer bước chân
        float peak_val = -999.0F;
        float trough_val = 999.0F;
        size_t peak_idx = 0;
        size_t trough_idx = 0;

        for (size_t i = 0; i < step_buf_len_; ++i) {
            if (step_accel_buf_[i] > peak_val) {
                peak_val = step_accel_buf_[i];
                peak_idx = i;
            }
            if (step_accel_buf_[i] < trough_val) {
                trough_val = step_accel_buf_[i];
                trough_idx = i;
            }
        }
        if (step_buf_len_ == 0) {
            peak_val = static_cast<float>(vertical_accel_m_s2);
            trough_val = static_cast<float>(min_during_step_m_s2_);
        }

        feat.peak_accel_m_s2 = peak_val;
        feat.trough_accel_m_s2 = trough_val;
        feat.peak_to_peak_m_s2 = peak_val - trough_val;
        feat.freefall_dip_m_s2 = trough_val;
        feat.freefall_duration_ms = static_cast<float>(freefall_time_s_ * 1000.0);

        if (peak_idx > trough_idx && step_buf_len_ > 0) {
            feat.rise_time_s = static_cast<float>(peak_idx - trough_idx) * static_cast<float>(dt_s);
        } else {
            feat.rise_time_s = static_cast<float>(time_since_trough_s_);
        }
        if (feat.rise_time_s <= 0.0F) {
            feat.rise_time_s = 0.15F;
        }

        if (feat.rise_time_s > 0.005F) {
            feat.vertical_jerk_m_s3 = feat.peak_to_peak_m_s2 / feat.rise_time_s;
            last_vertical_jerk_m_s3_ = feat.vertical_jerk_m_s3;
        }

        // Tính toán Temporal Energy Asymmetry Skewness
        feat.waveform_skewness = computeSkewness(step_accel_buf_.data(), step_buf_len_);
        last_skewness_ = feat.waveform_skewness;

        // Tính Step Delta Z & ZUPT Confidence
        feat.step_delta_z_m = static_cast<float>(step_displacement_z_m_);
        last_step_delta_z_ = feat.step_delta_z_m;

        const float z_abs = std::abs(feat.step_delta_z_m);
        feat.zupt_confidence = std::clamp(1.0F - std::abs(z_abs - 0.17F) / 0.35F, 0.2F, 1.0F) * orientation_confidence;
        last_zupt_confidence_ = feat.zupt_confidence;

        // 6. Tính tỷ số phương sai đứng/ngang và xác suất Stairness P(stair)
        float var_v = 1.0F;
        float var_h = 1.0F;
        const size_t win_len = window_filled_ ? kVarianceWindow : window_idx_;
        if (win_len > 10) {
            float mean_v = 0.0F;
            float mean_h = 0.0F;
            for (size_t i = 0; i < win_len; ++i) {
                mean_v += window_vert_accel_[i];
                mean_h += window_horiz_accel_[i];
            }
            mean_v /= static_cast<float>(win_len);
            mean_h /= static_cast<float>(win_len);

            float sum_sq_v = 0.0F;
            float sum_sq_h = 0.0F;
            for (size_t i = 0; i < win_len; ++i) {
                sum_sq_v += (window_vert_accel_[i] - mean_v) * (window_vert_accel_[i] - mean_v);
                sum_sq_h += (window_horiz_accel_[i] - mean_h) * (window_horiz_accel_[i] - mean_h);
            }
            var_v = sum_sq_v / static_cast<float>(win_len);
            var_h = sum_sq_h / static_cast<float>(win_len);
        }
        const float v_to_h_ratio = var_v / (var_h + 1e-3F);

        // [C1] p_stair hiệu chỉnh theo phân bố đo thực tế (40 file MotionSense túi quần
        // + 2 dataset cầm tay): biên độ p2p của THANG nằm trong dải 4.6-19 m/s², trong
        // khi đi bộ túi quần tạo biên độ LỚN HƠN thang (23-35) — giả định cũ "p2p càng
        // cao càng giống thang" là ngược thực tế. Kênh delta-Z bị loại khỏi p_stair:
        // tích phân reset-mỗi-bước mang DC bias cấu trúc −0.1..−0.6 m/bước (đo cả trên
        // đi phẳng) nên không đủ tin làm bằng chứng stair-vs-flat.
        float p_stair = static_cast<float>(config_.p_stair_base);
        if (feat.peak_to_peak_m_s2 >= 3.0F) {
            const float p_lo = std::clamp(static_cast<float>((feat.peak_to_peak_m_s2 - 3.0) / config_.p_lo_ramp), 0.0F, 1.0F);
            const float p_hi = std::clamp(static_cast<float>(1.0 - (feat.peak_to_peak_m_s2 - config_.p_hi_start) / config_.p_hi_falloff), 0.0F, 1.0F);
            const float p_vh = std::clamp(static_cast<float>((v_to_h_ratio - config_.p_vh_start) / config_.p_vh_scale), 0.0F, 1.0F);
            p_stair = static_cast<float>(config_.p_stair_base) + 0.70F * p_lo * p_hi + 0.10F * p_vh;

            // [C1] ĐÃ BỎ bonus freefall: đo thực tế, ngưỡng a_v < −2 m/s² kích hoạt
            // trên ~100% bước đi bộ thường (đáy sải chân) — không phải dấu hiệu
            // cầu thang, chỉ cộng điểm thang sai cho đi bộ.

            // Bảo vệ Flat walking [R1]: ZUPT delta-Z ≈ 0 là dấu hiệu KHÔNG có dịch
            // chuyển thẳng đứng thật.
            if (z_abs < static_cast<float>(config_.flat_guard_dz_m) && feat.freefall_duration_ms < 15.0F) {
                p_stair = std::min(p_stair, static_cast<float>(config_.flat_guard_cap));
            }
        }

        // 7. Log-Likelihood Softmax Fusion cho Hướng (Ascent vs Descent)
        // [C1] Trọng số hiệu chỉnh theo audit thực đo (D1/D2 cầm tay 100Hz, MotionSense
        // túi 50Hz): skewness là kênh bền nhất (thứ tự ascent>descent đúng trên cả 3
        // nguồn: D1 −0.11 vs −0.29; D2 +0.03 vs −0.11; MS −0.12 vs −0.21); delta-Z
        // mang DC bias cấu trúc nên hạ trọng số; rise-time thu hẹp ngưỡng theo phân bố
        // đo (thang 0.155-0.30, jog 0.145).
        // Kênh 1: Rise Time & Vertical Jerk (Trọng số 0.20)
        float f1_ascent = 0.50F;
        if (feat.rise_time_s <= static_cast<float>(config_.rise_descent_s)) {
            f1_ascent = 0.15F;
        } else if (feat.rise_time_s >= static_cast<float>(config_.rise_ascent_s) && feat.freefall_duration_ms < 60.0F) {
            f1_ascent = 0.85F;
        }
        const float f1_descent = 1.0F - f1_ascent;

        // Kênh 2: Freefall Dip & Duration (Trọng số 0.10)
        const float f2_ascent = (feat.freefall_duration_ms < 60.0F) ? 0.85F : 0.15F;
        const float f2_descent = 1.0F - f2_ascent;

        // Kênh 3: Cường độ Va đập Peak-to-Peak (Trọng số 0.10)
        const float f3_ascent = (feat.rise_time_s >= 0.22F && feat.freefall_duration_ms < 60.0F) ? 0.80F : ((feat.peak_to_peak_m_s2 <= 14.0F) ? 0.80F : (feat.peak_to_peak_m_s2 >= 22.0F ? 0.20F : 0.50F));
        const float f3_descent = 1.0F - f3_ascent;

        // Kênh 4: Step-wise ZUPT Delta Z (Trọng số 0.25 — có DC bias nên giữ nhạy vừa)
        const float f4_ascent = std::clamp(0.50F + static_cast<float>((feat.step_delta_z_m / 0.25) * config_.dz_gain), 0.15F, 0.85F);
        const float f4_descent = 1.0F - f4_ascent;

        // Kênh 5: Temporal Energy Asymmetry Skewness (Trọng số 0.20 — bền nhưng lệch vị trí đặt máy)
        const float f5_ascent = std::clamp(0.50F + static_cast<float>(feat.waveform_skewness * config_.skew_gain), 0.05F, 0.95F);
        const float f5_descent = 1.0F - f5_ascent;

        // Log-Likelihood
        // [C1] Trọng số chốt theo audit thực đo (freefall kích hoạt cả trên đi bộ;
        // dz/rise/p2p không tách được lớp trên dữ liệu thật; skewness là kênh duy
        // nhất có cực tính ascent>descent nhất quán — D1: −0.09 vs −0.29, D2: −0.01
        // vs −0.10, MS: +0.17 vs +0.02):
        const float log_L_A = static_cast<float>(config_.w_rise) * std::log(f1_ascent)
                            + static_cast<float>(config_.w_freefall) * std::log(f2_ascent)
                            + static_cast<float>(config_.w_p2p) * std::log(f3_ascent)
                            + static_cast<float>(config_.w_dz) * std::log(f4_ascent)
                            + static_cast<float>(config_.w_skew) * std::log(f5_ascent);

        const float log_L_D = static_cast<float>(config_.w_rise) * std::log(f1_descent)
                            + static_cast<float>(config_.w_freefall) * std::log(f2_descent)
                            + static_cast<float>(config_.w_p2p) * std::log(f3_descent)
                            + static_cast<float>(config_.w_dz) * std::log(f4_descent)
                            + static_cast<float>(config_.w_skew) * std::log(f5_descent);

        // Softmax
        const float max_log = std::max(log_L_A, log_L_D);
        const float exp_A = std::exp(log_L_A - max_log);
        const float exp_D = std::exp(log_L_D - max_log);
        const float p_ascent_norm = exp_A / (exp_A + exp_D);
        const float p_descent_norm = exp_D / (exp_A + exp_D);

        feat.ascent_likelihood = p_ascent_norm;
        feat.descent_likelihood = p_descent_norm;
        feat.ascent_score = p_ascent_norm * p_stair;
        feat.descent_score = p_descent_norm * p_stair;
        last_ascent_score_ = feat.ascent_score;
        last_descent_score_ = feat.descent_score;

        // Chiếu nghỉ (Landing Platform):
        // Chỉ kích hoạt khi đang ở trong vế thang (kConfirmed) và xuất hiện bước đi phẳng (P2P thấp và tỷ số đứng/ngang giảm)
        // [A6] Đảo hướng TƯƠNG ĐỐI trong phiên: skewness lệch khỏi EMA cụm hiện
        // hành > 0.15 trong 3 bước liên tiếp -> đảo chiều + re-seed cụm.
        // (Hướng tuyệt đối qua softmax giữ nguyên; kênh này hiệu chỉnh theo PHIÊN —
        // skewness tuyệt đối lệch theo vị trí đặt máy nhưng trong 1 phiên lên-xuống
        // độ lệch giữa 2 pha ổn định: đo D1 −0.09 vs −0.29, D2 −0.01 vs −0.10.)
        if (state_.phase == FlightPhase::kConfirmed) {
            const float skew = feat.waveform_skewness;
            if (!session_skew_seeded_) {
                session_skew_ema_ = skew;
                session_skew_seeded_ = true;
                session_skew_deviant_steps_ = 0;
                session_skew_seed_dir_ = (state_.activity == StairActivity::kAscent) ? 1 : -1;
            } else if (std::abs(skew - session_skew_ema_) > 0.15F) {
                session_skew_deviant_steps_++;
                if (session_skew_deviant_steps_ >= 3) {
                    const int current_dir = (state_.activity == StairActivity::kAscent) ? 1 : -1;
                    if (current_dir == session_skew_seed_dir_) {
                        // Hướng hiện hành vẫn là hướng seed -> đảo (kênh này là
                        // bằng chứng đảo hướng; anti-flip có thể đã đổi trước đó —
                        // khi đó chỉ re-seed, không đảo lần nữa).
                        if (state_.activity == StairActivity::kAscent) {
                            state_.activity = StairActivity::kDescent;
                            consecutive_descent_steps_ = 2;
                            consecutive_ascent_steps_ = 0;
                        } else {
                            state_.activity = StairActivity::kAscent;
                            consecutive_ascent_steps_ = 2;
                            consecutive_descent_steps_ = 0;
                        }
                        state_.direction_confidence = 0.85F;
                    }
                    session_skew_ema_ = skew;  // re-seed cụm mới (cả 2 nhánh)
                    session_skew_seed_dir_ = -current_dir;
                    session_skew_deviant_steps_ = 0;
                }
            } else {
                session_skew_deviant_steps_ = 0;
                session_skew_ema_ += 0.20F * (skew - session_skew_ema_);
            }
        }
        // [A5] Landing = bước WALKING-like (p2p nhảy lên > 1.5x median vế và > 12
        // m/s²) LẤP LIỆP >= 3 bước khi đang trong vế thang. Cập nhật median-EMA
        // chỉ bằng các bước KHÔNG landing để không bị pha loãng.
        const bool walking_like_step =
            (feat.peak_to_peak_m_s2 > static_cast<float>(config_.landing_p2p_jump_ratio * flight_p2p_median_ema_) &&
             feat.peak_to_peak_m_s2 > static_cast<float>(config_.landing_p2p_abs_min));
        if (walking_like_step && state_.phase == FlightPhase::kConfirmed) {
            consecutive_landing_steps_++;
        } else {
            consecutive_landing_steps_ = 0;
            flight_p2p_median_ema_ = 0.85F * flight_p2p_median_ema_
                + 0.15F * feat.peak_to_peak_m_s2;  // EMA median vế thang
        }
        last_is_landing_ = (state_.phase == FlightPhase::kConfirmed &&
                            consecutive_landing_steps_ >= config_.landing_consecutive_steps);

        // [R1] Lưu delta-Z ZUPT của bước vào state cho pipeline gate
        state_.last_step_delta_z_m = feat.step_delta_z_m;
        state_.last_rise_time_s = feat.rise_time_s;
        state_.last_skewness = feat.waveform_skewness;
        state_.last_v_to_h_ratio = v_to_h_ratio;
        state_.last_peak_to_peak_m_s2 = feat.peak_to_peak_m_s2;
        state_.last_freefall_ms = feat.freefall_duration_ms;

        // Cập nhật FSM & Adaptive Riser
        updateFsm(p_stair, p_ascent_norm, p_descent_norm, true, static_cast<float>(dt_s));
        updateAdaptiveRiser(feat.step_delta_z_m, feat.zupt_confidence);

        // Reset ZUPT
        current_v_z_m_s_ = 0.0;
        step_displacement_z_m_ = 0.0;
        freefall_time_s_ = 0.0;
        step_buf_len_ = 0;
        is_in_rising_phase_ = false;
        min_during_step_m_s2_ = 999.0;
        time_since_peak_s_ = 0.0;
    }

    feat.ascent_score = last_ascent_score_;
    feat.descent_score = last_descent_score_;
    feat.vertical_jerk_m_s3 = last_vertical_jerk_m_s3_;
    feat.waveform_skewness = last_skewness_;
    feat.step_delta_z_m = last_step_delta_z_;
    feat.zupt_confidence = last_zupt_confidence_;
    feat.freefall_duration_ms = last_freefall_ms_;
    feat.is_landing_candidate = last_is_landing_;

    return feat;
}

void StairMotionDetector::updateFsm(float p_stair, float p_ascent, float p_descent, bool is_step_peak, float /*dt_s*/) noexcept {
    if (!is_step_peak) {
        return;
    }

    state_.p_stair = p_stair;
    state_.p_ascent = p_ascent;
    state_.p_descent = p_descent;
    state_.p_unknown = std::max(0.0F, 1.0F - p_stair);

    if (state_.activity == StairActivity::kPaused) {
        state_.activity = (consecutive_ascent_steps_ >= consecutive_descent_steps_) ? StairActivity::kAscent : StairActivity::kDescent;
    }

    // [C1] Thoát lock-on: kConfirmed cũ bỏ qua p_stair thấp vĩnh viễn (chỉ reset khi
    // đứng nghỉ 1.8s) — hệ quả là sau khi xuống hết thang, đi bộ trên hành lang
    // vẫn bị coi là cầu thang. Đếm bước p_stair thấp liên tiếp -> thoát về kUnknown.
    if (p_stair < 0.35F && state_.phase == FlightPhase::kConfirmed) {
        consecutive_low_p_stair_steps_++;
        if (consecutive_low_p_stair_steps_ >= config_.low_p_exit_steps) {
            state_.activity = StairActivity::kNone;
            state_.phase = FlightPhase::kUnknown;
            state_.flight_step_count = 0;
            state_.flight_confidence = 0.0F;
            consecutive_ascent_steps_ = 0;
            consecutive_descent_steps_ = 0;
            consecutive_low_p_stair_steps_ = 0;
            return;
        }
    } else {
        consecutive_low_p_stair_steps_ = 0;
    }

    if (p_stair < 0.35F && state_.phase != FlightPhase::kConfirmed) {
        return;
    }

    state_.flight_step_count++;

    if (last_is_landing_) {
        state_.activity = StairActivity::kLanding;
        state_.phase = FlightPhase::kLandingTransition;
        return;
    }

    // Logic Hysteresis
    if (p_ascent > 0.55F) {
        consecutive_ascent_steps_++;
        consecutive_descent_steps_ = 0;
    } else if (p_descent > 0.55F) {
        consecutive_descent_steps_++;
        consecutive_ascent_steps_ = 0;
    }

    if (state_.phase == FlightPhase::kUnknown || state_.phase == FlightPhase::kCandidate) {
        if (consecutive_ascent_steps_ >= 2) {
            state_.activity = StairActivity::kAscent;
            state_.phase = FlightPhase::kConfirmed;
            state_.direction_confidence = 0.90F;
            state_.flight_confidence = std::min(1.0F, state_.flight_confidence + 0.30F);  // [R1]
        } else if (consecutive_descent_steps_ >= 2) {
            state_.activity = StairActivity::kDescent;
            state_.phase = FlightPhase::kConfirmed;
            state_.direction_confidence = 0.90F;
            state_.flight_confidence = std::min(1.0F, state_.flight_confidence + 0.30F);  // [R1]
        } else if (consecutive_ascent_steps_ == 1 && p_ascent > 0.65F) {
            state_.activity = StairActivity::kAscent;
            state_.phase = FlightPhase::kCandidate;
            state_.direction_confidence = 0.70F;
        } else if (consecutive_descent_steps_ == 1 && p_descent > 0.65F) {
            state_.activity = StairActivity::kDescent;
            state_.phase = FlightPhase::kCandidate;
            state_.direction_confidence = 0.70F;
        }
    } else if (state_.phase == FlightPhase::kLandingTransition) {
        // [A5] FIX bế ngưỡng FSM: kLandingTransition cũ KHÔNG có đường về — sau
        // chiếu nghỉ, bước thang trở lại không bao giờ khôi phục kConfirmed nên
        // hệ thống kẹt ở kLanding cho đến khi nghỉ 1.8s.
        if (!last_is_landing_) {
            state_.phase = FlightPhase::kConfirmed;
            state_.activity = (consecutive_ascent_steps_ >= consecutive_descent_steps_)
                ? StairActivity::kAscent : StairActivity::kDescent;
            state_.direction_confidence = 0.85F;
        }
    } else if (state_.phase == FlightPhase::kConfirmed) {
        if (state_.activity == StairActivity::kPaused) {
            state_.activity = (consecutive_ascent_steps_ >= consecutive_descent_steps_) ? StairActivity::kAscent : StairActivity::kDescent;
        }

        // Anti-flip: 2 bước liên tiếp ngược chiều với p > 0.65 (đo thực tế: đổi vế
        // ở chiếu nghỉ thường chỉ có 2-3 bước ngược chiều liền mạch)
        if (state_.activity == StairActivity::kAscent && consecutive_descent_steps_ >= config_.anti_flip_steps && p_descent > 0.65F) {
            state_.activity = StairActivity::kDescent;
            state_.direction_confidence = 0.85F;
        } else if (state_.activity == StairActivity::kDescent && consecutive_ascent_steps_ >= config_.anti_flip_steps && p_ascent > 0.65F) {
            state_.activity = StairActivity::kAscent;
            state_.direction_confidence = 0.85F;
        }
    }

    // Tích lũy độ cao Z
    if (state_.activity == StairActivity::kAscent) {
        state_.vertical_displacement_m += state_.step_height_m;
    } else if (state_.activity == StairActivity::kDescent) {
        state_.vertical_displacement_m -= state_.step_height_m;
    }
}

void StairMotionDetector::updateAdaptiveRiser(float delta_z, float confidence) noexcept {
    if (confidence < 0.40F) {
        return;
    }

    const float z_abs = std::abs(delta_z);
    if (z_abs >= 0.10F && z_abs <= 0.35F) {
        constexpr float kAlpha = 0.15F;
        state_.step_height_m = (1.0F - kAlpha) * state_.step_height_m + kAlpha * z_abs;
        state_.step_height_m = std::clamp(state_.step_height_m, 0.14F, 0.22F);
    }
}

} // namespace survival::kinematics
