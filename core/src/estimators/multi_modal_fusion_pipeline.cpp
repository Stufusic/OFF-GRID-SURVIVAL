#include "survival_core/estimators/multi_modal_fusion_pipeline.hpp"
#include "survival_core/integrity/integrity_guard.hpp"
#include "survival_core/math/angle_utils.hpp"
#include "survival_core/math/statistics.hpp"
#include <cmath>
#include <algorithm>

namespace survival::estimators {

MultiModalFusionPipeline::MultiModalFusionPipeline(const PipelineConfig& config) noexcept
    : config_(config)
    , eskf_(config.eskf_config)
    , stride_model_(config.stride_config)
    , pf_engine_(config.pf_config, config.random_seed)
    , filtered_baro_alt_m_(config.initial_baro_alt_m)
    , last_alt_sample_m_(config.initial_baro_alt_m)
{
}

void MultiModalFusionPipeline::initialize(const EastingNorthingAlt& initial_pos, const environment::DemMap& dem) noexcept {
    dem_ = dem;
    current_pdr_pos_ = initial_pos;
    current_pdr_cov_ = {1.0, 0.0, 0.0, 1.0};
    filtered_baro_alt_m_ = initial_pos.altitude_m;
    prev_baro_rate_m_ = initial_pos.altitude_m;
    last_alt_sample_m_ = initial_pos.altitude_m;
    total_steps_ = 0;
    last_timestamp_ns_ = 0;
    last_output_timestamp_ns_ = 0;
    has_initial_baro_ = false;
    last_baro_ts_ns_ = 0;
    last_adaptive_stride_m_ = 0.75;
    baro_bias_m_ = 0.0;  // [B2]

    activity_classifier_.reset();
    velocity_estimator_.reset();
    pf_engine_.initialize(initial_pos, config_.pf_config.init_pos_std_m);
    is_initialized_ = true;
}

void MultiModalFusionPipeline::updateAnchorPosition(const EastingNorthingAlt& new_anchor, double anchor_std_m) noexcept {
    const double delta_e = new_anchor.easting_m - current_pdr_pos_.easting_m;
    const double delta_n = new_anchor.northing_m - current_pdr_pos_.northing_m;
    const double delta_alt = new_anchor.altitude_m - current_pdr_pos_.altitude_m;
    const double delta_horiz = std::sqrt(delta_e * delta_e + delta_n * delta_n);

    // [A3] An toàn anchor: bản cũ hard-overwrite không gate — anchor hỏng hoàn
    // toàn teleport cả đám mây hạt lẫn PDR. Chỉ chặn nhảy phi lý > 1000m
    // (handoff GPS hợp lệ có thể xa hơn — test handoff 500m phải đi qua).
    if (delta_horiz > 1000.0) {
        return;
    }

    // [A2] Hiệu chuẩn stride K: innovation = quãng đường GNSS giữa 2 anchor vs
    // quãng đường PDR cùng khoảng. Chỉ khi đang đi/chạy (không phải vehicle) và
    // tỉ lệ hợp lý. EMA tau ~ 20 anchor, clamp [0.7, 1.4].
    if (config_.gnss_stride_calibration && has_prev_anchor_) {
        const double gnss_dist = std::sqrt(delta_e * delta_e + delta_n * delta_n);
        const double pdr_dist = pdr_dist_since_anchor_m_;
        if (pdr_dist > 2.0 && gnss_dist > 2.0 &&
            (activity_classifier_.getLastEstimate().current_activity == MotionActivity::kWalkingFlat ||
             activity_classifier_.getLastEstimate().current_activity == MotionActivity::kRunning)) {
            stride_scale_ = stride_scale_est_.update(gnss_dist / pdr_dist);
        }
    }
    has_prev_anchor_ = true;
    prev_anchor_ = new_anchor;
    pdr_dist_since_anchor_m_ = 0.0;

    // 1. Tịnh tiến toàn bộ đám mây hạt bảo toàn hình thái phân phối xác suất
    pf_engine_.shiftParticles(delta_e, delta_n, delta_alt);

    // 2. Tịnh tiến vị trí PDR
    current_pdr_pos_ = new_anchor;

    // 3. Cập nhật hiệp phương sai kết hợp độ bất định điểm neo mới
    const double s2 = std::max(1.0, anchor_std_m * anchor_std_m);
    current_pdr_cov_[0] += s2;
    current_pdr_cov_[3] += s2;

    // [A4] Neo dọc: khi GPS quay lại với Δcao độ lớn, hấp thụ vào baro_bias_m_
    // thay vì để chuỗi baro "đấu" lại (trôi cao độ tái phát sau mỗi lần mất GPS).
    if (std::abs(delta_alt) > 3.0) {
        baro_bias_m_ = std::clamp(baro_bias_m_ + delta_alt, -120.0, 120.0);
    }
}

std::optional<NavigationUncertaintyState> MultiModalFusionPipeline::processSensorFrame(const SensorFrame& raw_frame) noexcept {
    if (!is_initialized_) {
        initialize(EastingNorthingAlt{0.0, 0.0, 1000.0}, dem_);
    }

    // 0. Phòng vệ toàn vẹn dữ liệu (IntegrityGuard Firewall)
    SensorFrame frame = raw_frame;
    static_cast<void>(integrity_guard_.sanitizeSensorFrame(frame));
    if (!frame.has_imu) {
        return std::nullopt;
    }

    // 1. Đồng bộ thời gian đa tần số (Kẹp giới hạn nghiêm ngặt [0.001s, 0.20s])
    double dt = 0.01;
    if (last_timestamp_ns_ > 0 && frame.timestamp_ns > last_timestamp_ns_) {
        dt = static_cast<double>(frame.timestamp_ns - last_timestamp_ns_) * 1e-9;
        dt = std::clamp(dt, 0.001, 0.20);
    }
    last_timestamp_ns_ = frame.timestamp_ns;

    // 2. Fast Loop (100 Hz): ESKF 3D Attitude Estimation
    const Vector3f gyro{frame.gyro_rad_s[0], frame.gyro_rad_s[1], frame.gyro_rad_s[2]};
    const Vector3f accel{frame.accel_m_s2[0], frame.accel_m_s2[1], frame.accel_m_s2[2]};

    eskf_.predict(gyro, dt);
    eskf_.updateAccelerometer(accel);

    // [C3] ZARU: khi thiết bị quasi-static, hiệu chuẩn bias gyro (trước đây hàm
    // tồn tại nhưng KHÔNG AI gọi — bias gyro-Z không quan sát được khi không mag
    // nên trôi heading suốt phiên).
    // [C3-ZARU-EXP] if (eskf_.getAttitudeEstimate().is_quasi_static) {
    //     eskf_.updateZeroAngularRate();
    // }

    if (frame.has_mag) {
        const Vector3f mag{frame.mag_uT[0], frame.mag_uT[1], frame.mag_uT[2]};
        eskf_.updateMagnetometer(mag);
        latest_nis_ = eskf_.getLatestMagNis();
    }

    // 3. Khí áp kế (10 Hz): Lọc thông thấp cao độ vi sai không độ trễ khởi động
    bool baro_frame_now = false;
    if (frame.has_baro && frame.pressure_hpa > 300.0F && frame.pressure_hpa < 1100.0F) {
        const double raw_alt_m = 44330.0 * (1.0 - std::pow(static_cast<double>(frame.pressure_hpa) / 1013.25, 0.190295));
        if (!has_initial_baro_) {
            filtered_baro_alt_m_ = raw_alt_m;
            prev_baro_rate_m_ = raw_alt_m;
            last_alt_sample_m_ = raw_alt_m;
            has_initial_baro_ = true;
        } else {
            filtered_baro_alt_m_ = 0.95 * filtered_baro_alt_m_ + 0.05 * raw_alt_m;
        }
        baro_frame_now = true;
    }

    // [M1] baro_v_z chỉ tính trên CẶP MẪU BARO (10 Hz): trước đây prev bị refresh mỗi
    // frame 100 Hz -> mỗi frame baro baro_v_z = 10x tốc độ thật, 9/10 frame còn lại = 0.
    // Zero-order hold: giu toc do doc giua 2 mau baro (tuong tu classifier), het han
    // sau 3s mat baro de tranh giu gia tri cu vo thoi han
    double baro_v_z = 0.0;
    if (baro_frame_now) {
        const double dt_baro = static_cast<double>(frame.timestamp_ns - last_baro_ts_ns_) * 1e-9;
        if (dt_baro > 0.005 && dt_baro < 2.0) {
            last_baro_rate_m_s_ = (filtered_baro_alt_m_ - prev_baro_rate_m_) / dt_baro;
        }
        prev_baro_rate_m_ = filtered_baro_alt_m_;
        last_baro_ts_ns_ = frame.timestamp_ns;
        baro_v_z = last_baro_rate_m_s_;
    } else if (has_initial_baro_ && last_baro_ts_ns_ > 0 &&
               frame.timestamp_ns - last_baro_ts_ns_ <= 3000000000ULL) {
        baro_v_z = last_baro_rate_m_s_;
    }

    // 4. Trích xuất gia tốc thẳng đứng thực tế trong hệ ENU
    const double a_v = eskf_.getVerticalAcceleration(accel);

    // 5. Nhận diện bước chân qua Butterworth Bandpass Filter
    const bool is_step_detected = step_detector_.processSample(a_v, frame.timestamp_ns);
    const auto step_feat = step_detector_.getLastStepFeatures();
    const double delta_h = is_step_detected
        ? (filtered_baro_alt_m_ - last_alt_sample_m_)
        : (baro_v_z * dt);

    const auto att = eskf_.getAttitudeEstimate();

    // Tính toán cadence thực tế: nếu chưa từng có bước chân hoặc quá 1.5s không bước -> cadence = 0
    float effective_cadence_hz = 0.0F;
    if (step_detector_.getTotalStepCount() > 0 &&
        frame.timestamp_ns >= step_detector_.getLastStepTimestampNs() &&
        (frame.timestamp_ns - step_detector_.getLastStepTimestampNs()) <= 1500000000ULL) {
        effective_cadence_hz = static_cast<float>(step_feat.cadence_hz);
    }

    const float prev_speed_m_s = static_cast<float>(velocity_estimator_.getFilteredSpeed());

    // 6. Phân Loại Hoạt Động & Ước Lượng Vận Tốc Tức Thời 100Hz
    // [B4] Tốc độ cho classifier: ưu tiên GNSS Doppler khi hợp lệ — khởi động được
    // cycling/cable gate (trước đây tốc độ chỉ đến từ velocity estimator nên
    // xe đạp/cáp treo không bao giờ đạt ngưỡng 3.0/2.0 m/s)
    const float speed_for_classifier = (frame.gnss_speed_m_s >= 0.0F) ? frame.gnss_speed_m_s : prev_speed_m_s;
    const auto activity_est = activity_classifier_.processSample(
        frame,
        att.quaternion,
        effective_cadence_hz,
        is_step_detected,
        delta_h,
        frame.gnss_speed_m_s,
        dt,
        speed_for_classifier
    );

    const double fwd_accel = accel.y; // gia tốc tiến trong body frame
    const auto vel_out = velocity_estimator_.update(
        activity_est.current_activity,
        att.euler.yaw_rad,
        static_cast<double>(gyro.z),
        static_cast<double>(fwd_accel),
        last_adaptive_stride_m_,  // [M2] sải bước thích ứng (cache từ bước gần nhất)
        static_cast<double>(effective_cadence_hz),
        activity_est.stair_state.estimated_riser_m,
        static_cast<float>(baro_v_z),
        frame.gnss_speed_m_s,  // [B4]
        activity_est.vehicle_state.is_stopped,
        dt
    );

    // 7. Chính sách chuyển động cho các chế độ phi đi bộ (Per-Mode Motion Policies)
    const auto act = activity_est.current_activity;
    if (act == MotionActivity::kStationary) {
        // ZUPT lock: vị trí được giữ nguyên tuyệt đối, không dịch chuyển
    } else if (act == MotionActivity::kElevator) {
        // Thang máy: khóa vận tốc ngang, chỉ tích phân cao độ thẳng đứng
        const double dz = vel_out.vertical_speed_m_s * dt;
        current_pdr_pos_.altitude_m += dz;
        current_pdr_cov_[0] += 0.05 * dt;
        current_pdr_cov_[3] += 0.05 * dt;
        pf_engine_.propagateContinuousMotion(0.0, 0.0, dz, 0.0, dt, act);
    } else if (act == MotionActivity::kEscalator) {
        // Thang cuốn: vận tốc thấp, bám sát chiều dốc thang
        const double de = vel_out.velocity_enu_m_s.x * dt;
        const double dn = vel_out.velocity_enu_m_s.y * dt;
        const double dz = vel_out.vertical_speed_m_s * dt;
        current_pdr_pos_.easting_m += de;
        current_pdr_pos_.northing_m += dn;
        current_pdr_pos_.altitude_m += dz;
        current_pdr_cov_[0] += 0.10 * dt;
        current_pdr_cov_[3] += 0.10 * dt;
        pf_engine_.propagateContinuousMotion(de, dn, dz, vel_out.horizontal_speed_m_s, dt, act);
    } else if (act == MotionActivity::kCycling) {
        if (vel_out.horizontal_speed_m_s > 0.5) {
            const double de = vel_out.velocity_enu_m_s.x * dt;
            const double dn = vel_out.velocity_enu_m_s.y * dt;
            current_pdr_pos_.easting_m += de;
            current_pdr_pos_.northing_m += dn;
            current_pdr_cov_[0] += 0.20 * dt;
            current_pdr_cov_[3] += 0.20 * dt;
            pf_engine_.propagateContinuousMotion(de, dn, 0.0, vel_out.horizontal_speed_m_s, dt, act);
        }
    } else if (act == MotionActivity::kVehicleWheeled) {
        // Xe cơ giới: tích phân liên tục cả PDR và đám mây hạt
        if (!activity_est.vehicle_state.is_stopped) {
            const double de = vel_out.velocity_enu_m_s.x * dt;
            const double dn = vel_out.velocity_enu_m_s.y * dt;
            const double dz = vel_out.vertical_speed_m_s * dt;
            current_pdr_pos_.easting_m += de;
            current_pdr_pos_.northing_m += dn;
            current_pdr_pos_.altitude_m += dz;
            current_pdr_cov_[0] += 0.35 * dt;
            current_pdr_cov_[3] += 0.35 * dt;
            pf_engine_.propagateContinuousMotion(de, dn, dz, vel_out.horizontal_speed_m_s, dt, act);
        }
    } else if (act == MotionActivity::kCableTransport) {
        // [M3] Cáp treo: cùng lớp vận chuyển liên tục với xe cơ giới — trước đây
        // kCableTransport không có policy khiến PDR và đám mây hạt đóng băng (lỗi Bug 1)
        if (vel_out.horizontal_speed_m_s > 0.5 || std::abs(vel_out.vertical_speed_m_s) > 0.2) {
            const double de = vel_out.velocity_enu_m_s.x * dt;
            const double dn = vel_out.velocity_enu_m_s.y * dt;
            const double dz = vel_out.vertical_speed_m_s * dt;
            current_pdr_pos_.easting_m += de;
            current_pdr_pos_.northing_m += dn;
            current_pdr_pos_.altitude_m += dz;
            current_pdr_cov_[0] += 0.30 * dt;
            current_pdr_cov_[3] += 0.30 * dt;
            pf_engine_.propagateContinuousMotion(de, dn, dz, vel_out.horizontal_speed_m_s, dt, act);
        }
    }

    // 8. Xử lý khi có bước chân người đi bộ / chạy
    if (is_step_detected) {
        total_steps_++;
        last_alt_sample_m_ = filtered_baro_alt_m_;

        auto features = step_feat;
        features.terrain_slope_rad = last_pdr_slope_rad_;

        auto stride_res = stride_model_.estimateStride(features);  // [A2] mutable để áp scale K
        // [A2] Áp hệ số hiệu chuẩn K: mọi consumer phía sau (PDR, PF, velocity)
        // đều qua điểm chặn duy nhất này.
        const double scaled_mean = stride_res.mean_length_m * stride_scale_;
        stride_res.mean_length_m = std::clamp(scaled_mean, 0.15, 2.0);
        stride_res.variance_m2 *= stride_scale_ * stride_scale_;
        last_adaptive_stride_m_ = stride_res.mean_length_m;  // [M2] cache cho velocity estimator
        last_pdr_slope_rad_ = std::clamp(std::atan2(delta_h, std::max(0.20, stride_res.mean_length_m)), -0.80, 0.80);

        StepEvent step_event{};
        step_event.timestamp_ns = frame.timestamp_ns;
        step_event.stride_mean_m = stride_res.mean_length_m;
        step_event.stride_variance_m2 = stride_res.variance_m2;
        step_event.heading_rad = att.euler.yaw_rad;
        // [C3c] Floor 0.01 -> 0.02 rad² (NEES 357 — heading quá tin)
        step_event.heading_variance_rad2 = std::max(0.02, att.heading_variance_rad2);
        step_event.vertical_displacement_m = delta_h;
        step_event.activity = activity_est.current_activity;
        // [A1] FIX dead-code: khối ×2.0 khi trên thang đứng TRƯỚC lệnh gán activity
        // nên step_event.activity luôn là default kWalkingFlat — không bao giờ chạy.
        // Xoay người mỗi bước trên thang lớn hơn hẳn đi bộ.
        if (step_event.activity == MotionActivity::kStairsAscent ||
            step_event.activity == MotionActivity::kStairsDescent) {
            step_event.heading_variance_rad2 *= 2.0;
        }

        // Tích hợp Động học Cầu thang Xác suất (Probabilistic Stair Motion Integration)
        const auto& stair_state = activity_classifier_.getStairMotionState();
        // [R1] Chỉ áp dụng động học bậc thang khi FSM HOẠT ĐỘNG XÁC NHẬN.
        // Trước đây stair_state.activity là đủ — đi phẳng/xuống không baro bị
        // cộng ảo +0.17 m độ cao mỗi bước.
        // Bằng chứng độc lập khi không baro: FSM cầu thang phải ĐÃ XÁC NHẬN
        // (flat-guard ở detector: cap p_stair 0.30 khi delta-Z ≈ 0 — đi bộ phẳng
        // Khi có baro: baro là bằng chứng độc lập đủ mạnh. Khi KHÔNG baro: hướng
        // FSM phải nhất quán với pitch ESKF (H1: + = đổ về trước) — leo cần pitch
        // không âm, xuống cần pitch không dương. Level-t attitude (pitch ≈ 0) cho
        // phép cả hai (không có bằng chứng trái chiều).
        // [C1] Gate pitch RELATIVE (pitch − nền hiệu chuẩn): pitch tuyệt đối vô nghĩa
        // khi máy nghiêng tĩnh 30-65° trong túi/tay — gate cũ chặn cả LÂN lẫn XUỐNG
        // suốt chuyến đi (đo D1: chỉ 15s/79s được cộng vertical).
        const double pitch_rel_rad = att.euler.pitch_rad - activity_classifier_.getBaselinePitchRad();
        double stair_evidence_ok = false;
        if (frame.has_baro) {
            stair_evidence_ok = true;
        } else if (activity_classifier_.isStairSustained() &&
                   stair_state.activity == kinematics::StairActivity::kAscent &&
                   pitch_rel_rad >= -0.05) {
            stair_evidence_ok = true;
        } else if (activity_classifier_.isStairSustained() &&
                   stair_state.activity == kinematics::StairActivity::kDescent &&
                   pitch_rel_rad <= 0.05) {
            stair_evidence_ok = true;
        }
        const bool stair_confirmed_ascent = (step_event.activity == MotionActivity::kStairsAscent) ||
            (stair_state.activity == kinematics::StairActivity::kAscent && stair_evidence_ok);
        const bool stair_confirmed_descent = (step_event.activity == MotionActivity::kStairsDescent) ||
            (stair_state.activity == kinematics::StairActivity::kDescent && stair_evidence_ok);
        if (stair_confirmed_ascent) {
            step_event.activity = MotionActivity::kStairsAscent;
            step_event.stride_mean_m = kStairTreadM; // [A1] thống nhất hằng số dùng chung
            if (!frame.has_baro || std::abs(delta_h) < 0.05) {
                step_event.vertical_displacement_m = stair_state.step_height_m;
            }
        } else if (stair_confirmed_descent) {
            step_event.activity = MotionActivity::kStairsDescent;
            step_event.stride_mean_m = kStairTreadM; // [A1] thống nhất hằng số dùng chung
            if (!frame.has_baro || std::abs(delta_h) < 0.05) {
                step_event.vertical_displacement_m = -stair_state.step_height_m;
            }
        }

        // Cập nhật quỹ đạo PDR quán tính cơ sở (Dead Reckoning)
        const double sin_psi = std::sin(step_event.heading_rad);
        const double cos_psi = std::cos(step_event.heading_rad);
        current_pdr_pos_.easting_m += step_event.stride_mean_m * sin_psi;
        current_pdr_pos_.northing_m += step_event.stride_mean_m * cos_psi;
        current_pdr_pos_.altitude_m += step_event.vertical_displacement_m;
        // [A2] Tích lũy quãng đường PDR giữa 2 anchor cho hiệu chuẩn K
        pdr_dist_since_anchor_m_ += step_event.stride_mean_m;

        // Lan truyền Hiệp phương sai vị trí PDR 2D
        const double s_L2 = step_event.stride_variance_m2;
        const double s_psi2 = step_event.heading_variance_rad2;
        const double L = step_event.stride_mean_m;
        const double dp_ee = s_L2 * sin_psi * sin_psi + (L * L * s_psi2) * cos_psi * cos_psi;
        const double dp_nn = s_L2 * cos_psi * cos_psi + (L * L * s_psi2) * sin_psi * sin_psi;
        const double dp_en = (s_L2 - L * L * s_psi2) * sin_psi * cos_psi;

        current_pdr_cov_[0] += dp_ee;
        current_pdr_cov_[1] += dp_en;
        current_pdr_cov_[2] += dp_en;
        current_pdr_cov_[3] += dp_nn;

        // Lan truyền và Cập nhật Bộ lọc Hạt TRN
        pf_engine_.predict(step_event);

        std::optional<double> opt_sun_az = std::nullopt;
        if (frame.has_optical_sun && frame.optical_confidence > 0.30) {
            opt_sun_az = static_cast<double>(frame.optical_sun_azimuth_rad);
        }

        pf_engine_.updateWeights(
            dem_,
            filtered_baro_alt_m_ - baro_bias_m_,  // [B2] baro da khử bias QNH
            last_pdr_slope_rad_,
            att.euler.yaw_rad,
            opt_sun_az,
            frame.timestamp_ns
        );
    }

    // 9. Điều kiện phát trạng thái (Emission Condition)
    bool should_emit = is_step_detected;
    if (!should_emit) {
        if (last_output_timestamp_ns_ == 0) {
            should_emit = true;
        } else if (act == MotionActivity::kStationary) {
            // Nhịp tim 1 Hz khi đứng yên
            if (frame.timestamp_ns - last_output_timestamp_ns_ >= 1000000000ULL) {
                should_emit = true;
            }
        } else if (act == MotionActivity::kVehicleWheeled || act == MotionActivity::kCycling ||
                   act == MotionActivity::kElevator || act == MotionActivity::kEscalator ||
                   act == MotionActivity::kCableTransport) {
            // Nhịp 5 Hz (200 ms) cho các chế độ vận chuyển cơ giới
            if (frame.timestamp_ns - last_output_timestamp_ns_ >= 200000000ULL) {
                should_emit = true;
            }
        } else if (frame.timestamp_ns - last_output_timestamp_ns_ >= 1000000000ULL) {
            // Fallback: Tối thiểu 1 Hz đảm bảo không đóng băng UI / Blackbox
            should_emit = true;
        }
    }

    if (should_emit) {
        last_output_timestamp_ns_ = frame.timestamp_ns;

        auto uncertainty_state = pf_engine_.extractUncertaintyState(frame.timestamp_ns);
        uncertainty_state.flags.imu_healthy = (eskf_.getImuHealth() > 0.5F);
        uncertainty_state.flags.baro_healthy = frame.has_baro;
        uncertainty_state.flags.mag_anomaly = att.is_mag_anomalous;
        uncertainty_state.flags.celestial_available = frame.has_optical_sun;
        uncertainty_state.flags.zupt_active = (act == MotionActivity::kStationary);

        uncertainty_state.motion_activity = act;
        uncertainty_state.activity_confidence = activity_est.fused_confidence;

        if (act == MotionActivity::kStationary) {
            uncertainty_state.instantaneous_speed_m_s = 0.0F;
            uncertainty_state.vertical_speed_m_s = 0.0F;
        } else {
            uncertainty_state.instantaneous_speed_m_s = static_cast<float>(vel_out.horizontal_speed_m_s);
            uncertainty_state.vertical_speed_m_s = static_cast<float>(vel_out.vertical_speed_m_s);
        }
        uncertainty_state.position_source = PositionSource::kPdrTrn;

        // [B2] Bias áp kế QNH (thời tiết): baro tính từ 1013.25 hPa cố định trong khi
        // QNH thật trôi 10-50 m khi bão -> likelihood cao độ kéo đám mây tới SAI cao
        // độ vĩnh viễn. Ước lượng bias bằng EMA chậm khi filter TỰ TIN (1 cụm +
        // N_eff đủ) và DEM hợp lệ; gate |innov| ≤ 40 m chặn outlier.
        {
            const double n_eff_ratio = pf_engine_.getEffectiveParticleCount() /
                std::max(1.0, static_cast<double>(config_.pf_config.num_particles));
            const bool pf_confident = (uncertainty_state.active_clusters_count == 1) && (n_eff_ratio >= 0.3);
            double emit_dt_s = 0.0;
            if (latest_canonical_state_.timestamp_ns > 0 &&
                frame.timestamp_ns > latest_canonical_state_.timestamp_ns) {
                emit_dt_s = static_cast<double>(frame.timestamp_ns - latest_canonical_state_.timestamp_ns) * 1e-9;
            }
            if (pf_confident && emit_dt_s > 0.0 && emit_dt_s < 10.0) {
                const auto& b2_c0 = uncertainty_state.clusters[0];
                const auto dem_alt = dem_.getElevationBilinear(b2_c0.center_easting_m, b2_c0.center_northing_m);
                if (dem_alt.has_value()) {
                    const double innov = filtered_baro_alt_m_ - (dem_alt.value() + baro_bias_m_);
                    if (std::abs(innov) <= 40.0) {  // QNH cuoc do co the ±40-50 m
                        baro_bias_m_ += 0.01 * innov * emit_dt_s;  // tau ≈ 100 s
                        baro_bias_m_ = std::clamp(baro_bias_m_, -120.0, 120.0);
                    }
                }
            }
        }

        // 10. Giám sát toàn vẹn độc lập (IntegrityMonitor Audit)
        // [M4] Audit SO SÁNH frame hiện tại với mốc PHÁT CUỐI (canonical cũ) TRƯỚC khi
        // ghi đè canonical — trước đây so state với chính nó (cùng timestamp) nên
        // kFaultEstimatorStale là dead code. Nhịp emit fallback 1 Hz nên khoảng cách
        // thực tế <= ~1.1 s < ngưỡng 1.5 s: không có false positive.
        if (latest_canonical_state_.timestamp_ns > 0) {
            latest_integrity_status_ = integrity_monitor_.auditState(latest_canonical_state_, frame.timestamp_ns);
        }

        // Áp dụng hợp đồng phân loại kDegraded / kLost
        uncertainty_state.flags.estimator_diverged = !latest_integrity_status_.position_valid ||
            ((latest_integrity_status_.fault_flags & integrity::kFaultCovarianceNonPsd) != 0);
        uncertainty_state.flags.estimator_stale = ((latest_integrity_status_.fault_flags & integrity::kFaultEstimatorStale) != 0);

        if (!latest_integrity_status_.is_healthy) {
            if ((latest_integrity_status_.fault_flags & (integrity::kFaultCovarianceNonPsd | integrity::kFaultNanInfDetected)) != 0) {
                uncertainty_state.nav_mode = NavigationMode::kLost;
            } else {
                uncertainty_state.nav_mode = NavigationMode::kDegraded;
            }
        } else {
            uncertainty_state.nav_mode = att.is_mag_anomalous ? NavigationMode::kDegraded : NavigationMode::kTrnAided;
        }

        // 11. Cập nhật trạng thái chính tắc (canonical) — [M5] gán ĐẦY ĐỦ các trường
        // (trước đây nav_mode/position_source/ce50/ce95/heading/integrity bị bỏ trống
        // khiến consumer luôn thấy trạng thái GPS-aided mặc định)
        latest_canonical_state_.timestamp_ns = frame.timestamp_ns;
        latest_canonical_state_.position_enu = current_pdr_pos_;
        const auto [wgs_lat_rad, wgs_lon_rad] = dem_.enuToLatLon(current_pdr_pos_.easting_m, current_pdr_pos_.northing_m);
        // enuToLatLon trả về RADIANS — trường *_deg bắt buộc nhân kRadToDeg (types.hpp)
        latest_canonical_state_.position_wgs84.latitude_deg = wgs_lat_rad * kRadToDeg;
        latest_canonical_state_.position_wgs84.longitude_deg = wgs_lon_rad * kRadToDeg;
        latest_canonical_state_.position_wgs84.altitude_m = current_pdr_pos_.altitude_m;
        latest_canonical_state_.attitude_q = att.quaternion;
        latest_canonical_state_.velocity_m_s = vel_out.velocity_enu_m_s;
        latest_canonical_state_.speed_m_s = (act == MotionActivity::kStationary) ? 0.0 : vel_out.horizontal_speed_m_s;
        latest_canonical_state_.vertical_speed_m_s = (act == MotionActivity::kStationary) ? 0.0 : vel_out.vertical_speed_m_s;
        latest_canonical_state_.position_cov_2d = current_pdr_cov_;
        latest_canonical_state_.attitude_cov_3x3 = att.attitude_cov_3x3;
        latest_canonical_state_.gyro_bias = eskf_.getGyroBias();
        {
            const auto& c0 = uncertainty_state.clusters[0];
            // CE50 suy từ bán trục CE95: a50 = a95 * sqrt(chi2_50 / chi2_95) ≈ 0.48102
            latest_canonical_state_.ce50_m = c0.ce95_semi_major_m * 0.48102;
            latest_canonical_state_.ce95_m = c0.ce95_semi_major_m;
            latest_canonical_state_.ellipse_heading_rad = c0.ellipse_heading_rad;
        }
        latest_canonical_state_.nav_mode = uncertainty_state.nav_mode;
        latest_canonical_state_.position_source = uncertainty_state.position_source;
        latest_canonical_state_.sensor_health = SensorHealthModel{
            .imu = eskf_.getImuHealth(),
            .gyro = eskf_.getImuHealth(),
            .accel = eskf_.getImuHealth(),
            .magnetometer = eskf_.getMagHealth(),
            .barometer = frame.has_baro ? 1.0F : 0.0F,
            .gps = 1.0F
        };
        latest_canonical_state_.global_navigation_score = uncertainty_state.global_navigation_score;
        latest_canonical_state_.motion_activity = activity_est;
        latest_canonical_state_.integrity = latest_integrity_status_;

        return uncertainty_state;
    }

    return std::nullopt;
}

} // namespace survival::estimators
