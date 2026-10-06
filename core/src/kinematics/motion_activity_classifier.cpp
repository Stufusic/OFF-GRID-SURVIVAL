#include "survival_core/kinematics/motion_activity_classifier.hpp"
#include "survival_core/math/frame_conventions.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace survival::kinematics {

MotionActivityClassifier::MotionActivityClassifier(const MotionActivityClassifierConfig& config) noexcept
    : config_(config)
{
    reset();
}

void MotionActivityClassifier::reset() noexcept {
    stair_detector_.reset();
    stair_geometry_.reset();
    vehicle_detector_.reset();
    vertical_transport_.reset();
    fsm_.reset();
    last_estimate_ = MotionActivityEstimate{};
    current_horizontal_speed_ = 0.0F;
    prev_baro_alt_ = 0.0F;
    prev_baro_timestamp_ns_ = 0;
    baro_vertical_rate_ = 0.0F;
    has_baro_datum_ = false;
    baseline_pitch_rad_ = 0.0;
    baseline_pitch_samples_ = 0;
    has_baseline_pitch_ = false;
    cadence_hist_[0] = cadence_hist_[1] = cadence_hist_[2] = 0.0F;
    cadence_hist_count_ = 0;
    in_stair_session_ = false;
    last_stair_step_ts_ns_ = 0;
    session_landing_recorded_ = false;
    last_detector_stair_dir_ = 0;
}

MotionActivityEstimate MotionActivityClassifier::processSample(
    const SensorFrame& frame,
    const Quaterniond& attitude_q,
    float human_cadence_hz,
    bool is_step_peak,
    double delta_baro_alt_m,
    float gnss_speed_m_s,
    double dt_s,
    float estimated_horizontal_speed_m_s
) noexcept {
    if (dt_s <= 0.0 || dt_s > 0.5) {
        return last_estimate_;
    }

    current_horizontal_speed_ = estimated_horizontal_speed_m_s;

    // [C1] Cadence lọc median 3 bước: cadence per-step tức thời nhấp nháy
    // 1.4-2.6 Hz trên đi bộ (đo thực tế) — không lọc thì đường RUNNING phụ
    // nháy theo từng bước và FSM ngoài nhảy WALKING<->RUNNING liên tục.
    if (is_step_peak) {
        cadence_hist_[cadence_hist_count_ % 3] = human_cadence_hz;
        cadence_hist_count_++;
        last_step_frame_ts_ns_ = frame.timestamp_ns;
    }
    // History cadence chỉ hợp lệ trong 1.5s sau bước cuối (đồng bộ với hold window
    // của pipeline) — nếu không, giá trị cũ treo mãi khiến is_still sai vĩnh viễn.
    const bool cadence_hist_valid = last_step_frame_ts_ns_ != 0 &&
        (frame.timestamp_ns >= last_step_frame_ts_ns_) &&
        (frame.timestamp_ns - last_step_frame_ts_ns_) <= 1500000000ULL;
    float cadence_hz = human_cadence_hz;
    if (!cadence_hist_valid) {
        cadence_hist_count_ = 0;
    } else if (cadence_hist_count_ >= 3) {
        const float c0 = cadence_hist_[0], c1 = cadence_hist_[1], c2 = cadence_hist_[2];
        cadence_hz = std::max(std::min(c0, c1), std::min(std::max(c0, c1), c2));
    } else if (cadence_hist_count_ == 2) {
        cadence_hz = 0.5F * (cadence_hist_[0] + cadence_hist_[1]);
    }

    const Vector3f accel{frame.accel_m_s2[0], frame.accel_m_s2[1], frame.accel_m_s2[2]};
    const Vector3f gyro{frame.gyro_rad_s[0], frame.gyro_rad_s[1], frame.gyro_rad_s[2]};

    // 1. Chiếu Gia Tốc Về Hệ Quy Chiếu Trọng Lực Chuẩn (Gravity-Aligned Frame Projection)
    const Eigen::Matrix3d R_nb = math::quaternionToRotationMatrix(attitude_q);
    const Eigen::Vector3d g_body = R_nb.transpose() * Eigen::Vector3d(0.0, 0.0, 1.0);

    const Eigen::Vector3d a_body(
        static_cast<double>(accel.x),
        static_cast<double>(accel.y),
        static_cast<double>(accel.z)
    );

    const double a_v_total = a_body.dot(g_body);
    const double a_v_dynamic = a_v_total - kGravityStandard;

    const Eigen::Vector3d a_h_vec = a_body - a_v_total * g_body;
    const double a_h_norm = a_h_vec.norm();

    // Trích xuất góc Pitch từ thái độ ESKF q_nb (Body -> Nav)
    // Frame BODY (docs/frame_conventions.md §II): +X = Phải, +Y = Trước, +Z = Lên.
    // Pitch vật lý = độ cao của trục TRƯỚC (+Y) so với phương ngang: sin(elev) = R(2,1) = 2(yz + wx).
    // Quy ước: pitch dương = trục trước chúc XUỐNG (thân đổ về trước) -> lấy phủ định elevation.
    // (Công thức cũ asin(2(wy − zx)) là độ cao trục PHẢI = nghiêng ngang — sai trục, audit H1.)
    const double fwd_elev_sin = 2.0 * (attitude_q.y * attitude_q.z + attitude_q.w * attitude_q.x);
    const double pitch_rad = -std::asin(std::clamp(fwd_elev_sin, -1.0, 1.0));

    const float gyro_norm = std::sqrt(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
    const bool is_still = (std::abs(a_body.norm() - kGravityStandard) < 0.35 && gyro_norm < 0.12F && cadence_hz < 0.2F);

    // [C1] Hiệu chuẩn pitch nền khi đứng yên: điện thoại trong túi nghiêng tĩnh
    // 45-65° (đo trên MotionSense) nên ngưỡng pitch TUYỆT ĐỐI ±0.08 rad vô nghĩa —
    // mọi quyết định hướng dùng RELATIVE pitch = pitch − baseline. Baseline hội tụ
    // nhanh ở 3s đứng yên đầu (EMA α=0.08/mẫu) và tiếp tục bám khi có lần đứng yên sau.
    if (is_still && gyro_norm < 0.05F) {
        const double kBaselineAlpha = config_.baseline_pitch_alpha;
        baseline_pitch_rad_ = (1.0 - kBaselineAlpha) * baseline_pitch_rad_ + kBaselineAlpha * pitch_rad;
        baseline_pitch_samples_++;
        has_baseline_pitch_ = baseline_pitch_samples_ >= config_.baseline_pitch_min_samples; // ≥0.3s đứng yên
    }
    const double rel_pitch = has_baseline_pitch_ ? (pitch_rad - baseline_pitch_rad_) : pitch_rad;

    // 2. Tính tốc độ biến thiên khí áp (Vertical Baro Rate)
    // [C4] Expiry: baro mất mẫu > 2s thì đặt lại rate = 0 — bản cũ dùng giá trị
    // EMA cũ vô thời hạn (pipeline đã có zero-order-hold 3s, classifier thì không).
    if (has_baro_datum_ && (!frame.has_baro || frame.pressure_hpa <= 300.0F)) {
        const uint64_t stale_ns = frame.timestamp_ns - prev_baro_timestamp_ns_;
        if (prev_baro_timestamp_ns_ != 0 && stale_ns > 2000000000ULL) {
            baro_vertical_rate_ = 0.0F;
        }
    }
    if (frame.has_baro && frame.pressure_hpa > 300.0F) {
        const double baro_alt = 44330.0 * (1.0 - std::pow(frame.pressure_hpa / kStandardPressureHpa, 0.190295));
        // [M6] Gate theo "đã có mẫu baro" thay vì theo ĐỘ CAO (baro_alt có thể ≈ 0 hoặc
        // ÂM ở ven biển/áp cao — gate cũ prev_baro_alt_ > 1.0 vô hiệu hóa baro vĩnh viễn)
        if (!has_baro_datum_) {
            has_baro_datum_ = true;
            prev_baro_alt_ = static_cast<float>(baro_alt);
            prev_baro_timestamp_ns_ = frame.timestamp_ns;
        } else if (frame.timestamp_ns > prev_baro_timestamp_ns_) {
            const double baro_dt = static_cast<double>(frame.timestamp_ns - prev_baro_timestamp_ns_) * 1e-9;
            if (baro_dt >= 0.005 && baro_dt <= 1.0) {
                const float raw_rate = static_cast<float>((baro_alt - prev_baro_alt_) / baro_dt);
                baro_vertical_rate_ = 0.70F * baro_vertical_rate_ + 0.30F * raw_rate;
            }
            prev_baro_alt_ = static_cast<float>(baro_alt);
            prev_baro_timestamp_ns_ = frame.timestamp_ns;
        }
    }

    // 3. Chạy các bộ nhận diện thành phần
    const StairDetectorFeatures stair_feat = stair_detector_.processSample(a_v_dynamic, a_h_norm, dt_s, is_step_peak);
    const VehicleFeatures veh_feat = vehicle_detector_.processSample(accel, gnss_speed_m_s, human_cadence_hz, dt_s);
    const VerticalTransportScores vert_scores = vertical_transport_.processSample(
        baro_vertical_rate_,
        static_cast<float>(a_v_dynamic),
        human_cadence_hz,
        current_horizontal_speed_,
        veh_feat.hf_ratio,
        dt_s
    );

    // 4. Đánh giá bảng điểm bằng chứng cho 11 hoạt động (Candidate Scores)
    std::array<float, 12> scores{};
    scores.fill(0.0F);

    // [0] kStationary: Người dùng đứng yên nhưng không nằm trong thang máy/thang cuốn đang chuyển động
    if (is_still && veh_feat.vehicle_score < 0.3F && vert_scores.elevator_score < 0.5F && vert_scores.escalator_score < 0.5F) {
        scores[0] = 0.95F;
    }

    // [3] kStairsAscent & [4] kStairsDescent
    const auto& stair_state = stair_detector_.getMotionState();
    scores[3] = stair_feat.ascent_score;
    scores[4] = stair_feat.descent_score;

    // [C1] Cập nhật cửa sổ p_stair duy trì (theo bước)
    if (is_step_peak) {
        recent_p_stair_[recent_p_stair_idx_ % 8] = stair_state.p_stair; // = config_.stair_sustain_window
        recent_p_stair_idx_++;
        if (recent_p_stair_count_ < 8) recent_p_stair_count_++;
    }
    // [C1] Rule FRACTION thay vì mean: các bước landing (chiếu nghỉ) giữa các vế
    // thang có p_stair thấp và kéo mean rơi dưới ngưỡng — D1 có 18 chiếu nghỉ nên
    // mean-8 hầu như không bao giờ đạt. Đếm tỷ lệ bước "giống thang" (p_stair ≥
    // 0.60) trong cửa sổ: ≥ 5/8 = sustained.
    uint32_t p_stair_hit_count = 0;
    for (uint32_t i = 0; i < recent_p_stair_count_; ++i) {
        if (recent_p_stair_[i] >= static_cast<float>(config_.stair_sustain_threshold)) p_stair_hit_count++;
    }
    // [A2-fix] Cache cho getter isStairSustained() (pipeline gate thống nhất)
    last_stair_sustained_ = (recent_p_stair_count_ >= config_.stair_sustain_window) &&
                            (p_stair_hit_count >= config_.stair_sustain_hits);
    const bool stair_sustained = last_stair_sustained_;

    // [C1] KHÔNG có bằng chứng thang duy trì (và không có baro) -> triệt zero điểm
    // thang: điểm stair_feat thô trên đi bộ túi quần tới 0.4-0.76, nếu nạp vào FSM
    // ngoài thì temporal memory sẽ giữ nhãn stairs suốt cả lượt đi bộ.
    if (!stair_sustained && !(frame.has_baro || has_baro_datum_)) {
        scores[3] = 0.0F;
        scores[4] = 0.0F;
    }

    const bool baro_available = frame.has_baro || has_baro_datum_;
    // [S1] Yeu cau CA HAI tin hieu cung chieu (rate la EMA, delta la trich mau buoc):
    // truoc day dung OR nen nhieu ap ke roi vao band rate la boost stair ngay ->
    // di phang co nhieu bi phan loai sai o cac trang thai bien.
    const bool is_baro_ascent = (baro_vertical_rate_ >= static_cast<float>(config_.baro_rate_band_lo) && baro_vertical_rate_ <= static_cast<float>(config_.baro_rate_band_hi)) &&
                                (delta_baro_alt_m >= config_.baro_delta_lo && delta_baro_alt_m <= config_.baro_delta_hi);
    const bool is_baro_descent = (baro_vertical_rate_ <= -static_cast<float>(config_.baro_rate_band_lo) && baro_vertical_rate_ >= -static_cast<float>(config_.baro_rate_band_hi)) &&
                                 (delta_baro_alt_m <= -config_.baro_delta_lo && delta_baro_alt_m >= -config_.baro_delta_hi);

    if (baro_available && is_baro_ascent && cadence_hz > 0.8F) {
        scores[3] = std::max(scores[3] + 0.40F, 0.85F);
        scores[4] *= 0.15F;
    } else if (baro_available && is_baro_descent && cadence_hz > 0.8F) {
        scores[4] = std::max(scores[4] + 0.40F, 0.85F);
        scores[3] *= 0.15F;
    } else if (baro_available && std::abs(baro_vertical_rate_) < 0.10F && std::abs(delta_baro_alt_m) < 0.10) {
        scores[3] *= 0.15F;
        scores[4] *= 0.15F;
    } else if (!baro_available) {
        // Không có Baro (Inertial/Biomechanical Inference):
        // 1. Pitch TƯƠNG ĐỐI (đã khử nghiêng tĩnh) — chỉ là bằng chứng YẾU:
        //    pitch-only không được vượt 0.50 (dưới ngưỡng candidate 0.55), chỉ đủ
        //    định hướng khi đã có bằng chứng thang từ kênh khác. Chặn đúng kịch bản
        //    "điện thoại nghiêng tĩnh trong túi -> mọi chuyển động bị coi là descend".
        if (rel_pitch > config_.pitch_gate_rad && cadence_hz > 0.8F && (stair_feat.rise_time_s >= 0.15F || stair_feat.ascent_score > 0.35F)) {
            scores[3] = std::max(scores[3], 0.50F);
            scores[4] *= 0.60F;
        } else if (rel_pitch < -config_.pitch_gate_rad && cadence_hz > 0.8F && (stair_feat.rise_time_s <= 0.12F || stair_feat.descent_score > 0.35F)) {
            scores[4] = std::max(scores[4], 0.50F);
            scores[3] *= 0.60F;
        }

        // 2. Tích hợp trực tiếp từ StairMotionDetector FSM — chỉ khi phase đã
        //    kConfirmed (2 bước liên tiếp cùng chiều) VÀ nhịp bước thuộc dải thang
        //    (cadence < 2.2 Hz: chạy bộ 2.3+ Hz không được khóa FSM thang);
        //    kCandidate chỉ đủ điểm dưới ngưỡng candidate để không khóa sai sớm.
        if (stair_state.activity == StairActivity::kAscent && stair_state.phase == FlightPhase::kConfirmed && rel_pitch >= -config_.pitch_gate_rad && cadence_hz < config_.cadence_stair_cap && stair_sustained) {
            scores[3] = std::max(scores[3], 0.85F * (stair_state.direction_confidence > 0.0F ? stair_state.direction_confidence : 0.80F));
            scores[4] *= 0.15F;
        } else if (stair_state.activity == StairActivity::kDescent && stair_state.phase == FlightPhase::kConfirmed && rel_pitch <= config_.pitch_gate_rad && cadence_hz < config_.cadence_stair_cap && stair_sustained) {
            scores[4] = std::max(scores[4], 0.85F * (stair_state.direction_confidence > 0.0F ? stair_state.direction_confidence : 0.80F));
            scores[3] *= 0.15F;
        } else if (stair_state.activity == StairActivity::kAscent && stair_state.phase == FlightPhase::kCandidate) {
            scores[3] = std::max(scores[3], 0.50F * stair_state.direction_confidence);
        } else if (stair_state.activity == StairActivity::kDescent && stair_state.phase == FlightPhase::kCandidate) {
            scores[4] = std::max(scores[4], 0.50F * stair_state.direction_confidence);
        } else if (stair_state.activity == StairActivity::kPaused) {
            if (stair_state.phase == FlightPhase::kConfirmed) {
                if (stair_state.p_ascent > stair_state.p_descent) {
                    scores[3] = 0.75F;
                } else {
                    scores[4] = 0.75F;
                }
            }
        }

        // [R1] Giải mâu thuẫn hướng bằng RELATIVE pitch: FSM bảo leo nhưng thân
        // người ngửa ra sau so với tư thế nền thì triệt tiêu điểm ascent (và ngược lại).
        if (stair_state.activity == StairActivity::kAscent && rel_pitch < -config_.pitch_gate_rad) {
            scores[3] *= 0.20F;
        } else if (stair_state.activity == StairActivity::kDescent && rel_pitch > config_.pitch_gate_rad) {
            scores[4] *= 0.20F;
        }
    }

    // Nếu người dùng bước đi bằng phẳng (relative pitch gần 0, không có tín hiệu
    // leo dốc đứng): với pitch nền đã hiệu chuẩn, điều kiện này giờ_fire đúng trên
    // điện thoại nghiêng tĩnh trong túi.
    if (std::abs(rel_pitch) < config_.pitch_gate_rad && !is_baro_ascent && !is_baro_descent && stair_state.phase != FlightPhase::kConfirmed) {
        scores[3] *= static_cast<float>(config_.flat_suppress_factor);
        scores[4] *= static_cast<float>(config_.flat_suppress_factor);
    }

    // [C1] Candidate thang phải qua cửa bằng chứng DUY TRÌ: điểm stair_feat thô
    // (p_stair × hướng) trên đi bộ túi quần thường 0.4-0.76 — không gate sustained
    // thì đi bộ cứ bị coi là candidate và bị dìm khỏi nhãn WALKING.
    const bool is_stair_candidate = stair_sustained &&
        (scores[3] >= 0.55F || scores[4] >= 0.55F || stair_state.phase == FlightPhase::kConfirmed);
    // [C1] stair_confirmed phải tôn trọng nhịp bước (chạy 2.3+ Hz không phải nhịp
    // thang) VÀ bằng chứng thang phải DUY TRÌ (mean p_stair 6 bước ≥ 0.80) —
    // p_stair của đi bộ dao động thấp sẽ trả nhãn về WALKING thay vì khóa sai
    // cả một lượt đi bộ thành "xuống thang".
    const bool stair_confirmed = ((stair_state.phase == FlightPhase::kConfirmed && cadence_hz < static_cast<float>(config_.cadence_stair_cap) && stair_sustained) || is_baro_ascent || is_baro_descent);

    // [1] kWalkingFlat
    if (cadence_hz >= static_cast<float>(config_.cadence_walk_lo) && cadence_hz <= static_cast<float>(config_.cadence_walk_hi)) {
        scores[1] = stair_confirmed ? 0.20F : static_cast<float>(config_.walking_score);
    }

    // [2] kRunning (Chạy bộ bằng phẳng)
    // [C1] Chạy ưu tiên theo nhịp: cadence chạy (2.3+) không bị chặn bởi
    // "nghi ngờ thang chưa xác nhận" — trước đây 100% lượt jog bị dìm về stairs.
    if (!stair_confirmed) {
        if (cadence_hz > static_cast<float>(config_.cadence_run) || (cadence_hz > static_cast<float>(config_.cadence_run_secondary) && stair_feat.peak_to_peak_m_s2 > static_cast<float>(config_.p2p_run_secondary) && stair_feat.rise_time_s < 0.15F)) {
            if (stair_feat.rise_time_s < 0.15F) {
                scores[2] = 0.90F;
            } else {
                scores[2] = 0.75F;
            }
        }
    }

    // [5] kStairLanding
    const MotionActivity prev_act = fsm_.getCurrentActivity();
    if ((prev_act == MotionActivity::kStairsAscent || prev_act == MotionActivity::kStairsDescent || prev_act == MotionActivity::kStairLanding) &&
        stair_feat.is_landing_candidate) {
        scores[5] = 0.80F;
    }

    // [6] kEscalator
    scores[6] = vert_scores.escalator_score;

    // [7] kCycling
    scores[7] = vert_scores.cycling_score;

    // [8] kVehicleWheeled
    scores[8] = veh_feat.vehicle_score;

    // [9] kElevator
    scores[9] = vert_scores.elevator_score;

    // [10] kCableTransport
    scores[10] = vert_scores.cable_score;

    // 5. Cập nhật Máy Trạng Thái FSM có trễ (Hysteresis FSM)
    MotionActivityEstimate est = fsm_.update(scores, dt_s);

    // 6. Ghi nhận hình học bậc thang nếu đang ở chế độ cầu thang
    if (is_step_peak) {
        // [A5] Phiên thang + landing hồi tố: vế thang -> walking ngắn -> vế thang.
        // Dùng FSM detector (kConfirmed) thay vì nhãn — nhãn qua sustained-gate
        // phản ứng chậm hơn nhiều so với xác nhận 2 bước của detector.
        const auto& fsm_state = stair_detector_.getMotionState();
        const bool activity_is_stairs =
            (fsm_state.phase == FlightPhase::kConfirmed &&
             (fsm_state.activity == StairActivity::kAscent ||
              fsm_state.activity == StairActivity::kDescent));

        // [A6] Đảo hướng trong phiên: detector đảo ascent<->descent (skewness lệch
        // cụm 3 bước) — outer FSM cấm chuyển thẳng ascent<->descent nên phải đồng
        // bộ buộc, nếu không nhãn kẹt chiều cũ suốt phần còn lại của phiên.
        if (activity_is_stairs) {
            const int detector_dir = (fsm_state.activity == StairActivity::kAscent) ? 1 : -1;
            if (last_detector_stair_dir_ != 0 && detector_dir != last_detector_stair_dir_) {
                fsm_.forceActivity(detector_dir > 0 ? MotionActivity::kStairsAscent
                                                    : MotionActivity::kStairsDescent);
            }
            last_detector_stair_dir_ = detector_dir;
        } else {
            last_detector_stair_dir_ = 0;
        }
        if (activity_is_stairs) {
            if (!in_stair_session_) {
                if (last_stair_step_ts_ns_ != 0 &&
                    frame.timestamp_ns > last_stair_step_ts_ns_ &&
                    (frame.timestamp_ns - last_stair_step_ts_ns_) <= 8000000000ULL &&
                    !session_landing_recorded_) {
                    stair_geometry_.recordLandingStep();  // khoảng walking = chiếu nghỉ
                    session_landing_recorded_ = true;
                }
                in_stair_session_ = true;
            }
            last_stair_step_ts_ns_ = frame.timestamp_ns;
        } else if (in_stair_session_ &&
                   (est.current_activity == MotionActivity::kWalkingFlat ||
                    est.current_activity == MotionActivity::kStairLanding ||
                    est.current_activity == MotionActivity::kStationary)) {
            // vẫn trong phiên — chờ quay lại vế thang; quá 8s thì đóng phiên ở dưới
            if (frame.timestamp_ns > last_stair_step_ts_ns_ &&
                (frame.timestamp_ns - last_stair_step_ts_ns_) > 8000000000ULL) {
                in_stair_session_ = false;
            }
        } else if (in_stair_session_) {
            in_stair_session_ = false;
        }
        if (est.current_activity == MotionActivity::kStairsAscent) {
            stair_geometry_.recordStairStep(true, delta_baro_alt_m, frame.has_baro, (human_cadence_hz > 0.1F) ? 1.0 / human_cadence_hz : 0.5);
        } else if (est.current_activity == MotionActivity::kStairsDescent) {
            stair_geometry_.recordStairStep(false, delta_baro_alt_m, frame.has_baro, (human_cadence_hz > 0.1F) ? 1.0 / human_cadence_hz : 0.5);
        } else if (est.current_activity == MotionActivity::kStairLanding) {
            stair_geometry_.recordLandingStep();
        }
    }

    // 7. Điền thông tin bằng chứng đầy đủ (Explainable MotionEvidence)
    est.stair_state = stair_geometry_.getStairState();
    est.vehicle_state.subtype = (est.current_activity == MotionActivity::kCycling) ? VehicleSubtype::kBicycle : VehicleSubtype::kUnknown;
    est.vehicle_state.is_stopped = veh_feat.is_stopped;
    est.vehicle_state.gnss_speed_m_s = (gnss_speed_m_s >= 0.0F) ? gnss_speed_m_s : 0.0F;
    est.vehicle_state.hf_vibration_ratio = veh_feat.hf_ratio;
    est.vehicle_state.confidence = est.fused_confidence;

    est.evidence.cadence_hz = human_cadence_hz;
    est.evidence.vertical_energy = static_cast<float>(a_v_dynamic * a_v_dynamic);
    est.evidence.horizontal_energy = static_cast<float>(a_h_norm * a_h_norm);
    est.evidence.impact_score = stair_feat.peak_to_peak_m_s2;
    est.evidence.freefall_score = stair_feat.freefall_dip_m_s2;
    est.evidence.hf_vibration_ratio = veh_feat.hf_ratio;
    est.evidence.spectral_entropy = veh_feat.spectral_entropy;
    est.evidence.baro_vertical_rate = baro_vertical_rate_;
    est.evidence.gnss_speed = (gnss_speed_m_s >= 0.0F) ? gnss_speed_m_s : 0.0F;
    est.evidence.stair_score = std::max(stair_feat.ascent_score, stair_feat.descent_score);
    est.evidence.vehicle_score = veh_feat.vehicle_score;
    est.evidence.escalator_score = vert_scores.escalator_score;
    est.evidence.elevator_score = vert_scores.elevator_score;

    last_estimate_ = est;
    return est;
}

} // namespace survival::kinematics
