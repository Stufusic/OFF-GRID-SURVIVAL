#pragma once

#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/kinematics/stair_motion_detector.hpp"
#include "survival_core/kinematics/stair_geometry_estimator.hpp"
#include "survival_core/kinematics/vehicle_motion_detector.hpp"
#include "survival_core/kinematics/vertical_transport_detector.hpp"
#include "survival_core/kinematics/motion_transition_fsm.hpp"

namespace survival::kinematics {

// [C4] Cấu hình MotionActivityClassifier — mặc định = giá trị đã hiệu chỉnh
// trên MotionSense 360 recordings + 2 dataset tự thu (báo cáo 2026-09-28).
struct MotionActivityClassifierConfig {
    double pitch_gate_rad{0.08};       // ngưỡng relative pitch (đã khử nghiêng tĩnh)
    double baseline_pitch_alpha{0.08}; // EMA pitch nền khi đứng yên
    uint32_t baseline_pitch_min_samples{30};

    // Baro bands (rate EMA [m/s], delta theo bước [m])
    double baro_rate_band_lo{0.10};
    double baro_rate_band_hi{0.65};
    double baro_delta_lo{0.02};
    double baro_delta_hi{0.60};
    double baro_stale_s{2.0};          // mất mẫu quá lâu -> rate = 0

    // Bằng chứng thang duy trì: >= hits/window bước có p_stair >= threshold
    uint32_t stair_sustain_window{8};
    uint32_t stair_sustain_hits{5};
    double stair_sustain_threshold{0.60};  // 0.60: D1/D2 vertical tốt nhất (0.55 làm Z_end −4.8)

    // Cadence [Hz]
    double cadence_walk_lo{0.9};
    double cadence_walk_hi{2.3};
    double cadence_run{2.3};
    double cadence_run_secondary{2.0};
    double p2p_run_secondary{18.0};
    double cadence_stair_cap{2.2};

    // Điểm
    double stair_score_gate{0.55};
    double flat_suppress_factor{0.20};
    double walking_score{0.85};
};

class MotionActivityClassifier {
public:
    MotionActivityClassifier(const MotionActivityClassifierConfig& config = MotionActivityClassifierConfig{}) noexcept;

    /**
     * @brief Xử lý một khung cảm biến 100 Hz kết hợp thái độ định hướng từ ESKF
     * @param frame SensorFrame đa kênh (Gia tốc, con quay, từ kế, áp kế)
     * @param attitude_q Quaternion thái độ q_nb (Body -> Nav) từ ESKF
     * @param human_cadence_hz Nhịp bước chân người
     * @param is_step_peak true nếu mẫu này là đỉnh của một bước chân
     * @param delta_baro_alt_m Độ cao vi sai áp kế
     * @param gnss_speed_m_s Vận tốc từ GNSS Doppler (< 0 nếu mất GPS)
     * @param dt_s Bước thời gian [s]
     */
    [[nodiscard]] MotionActivityEstimate processSample(
        const SensorFrame& frame,
        const Quaterniond& attitude_q,
        float human_cadence_hz,
        bool is_step_peak,
        double delta_baro_alt_m,
        float gnss_speed_m_s,
        double dt_s,
        float estimated_horizontal_speed_m_s = 0.0F
    ) noexcept;

    void reset() noexcept;

    [[nodiscard]] const MotionActivityEstimate& getLastEstimate() const noexcept {
        return last_estimate_;
    }

    [[nodiscard]] const StairState& getStairState() const noexcept {
        return stair_geometry_.getStairState();
    }

    [[nodiscard]] const StairMotionState& getStairMotionState() const noexcept {
        return stair_detector_.getMotionState();
    }

    /// [A2-fix] Bằng chứng thang DUY TRÌ của khung gần nhất (pipeline dùng để
    /// thống nhất cổng kinematics cầu thang với cổng nhãn).
    [[nodiscard]] bool isStairSustained() const noexcept { return last_stair_sustained_; }

    /// [C1] Pitch nền đã hiệu chuẩn (rad) — 0.0 nếu chưa có đủ mẫu đứng yên.
    /// Pipeline dùng để biến các gate pitch TUYỆT ĐỐI thành RELATIVE pitch
    /// (điện thoại nghiêng tĩnh trong túi/tay làm gate tuyệt đối vô nghĩa).
    [[nodiscard]] double getBaselinePitchRad() const noexcept {
        return has_baseline_pitch_ ? baseline_pitch_rad_ : 0.0;
    }

private:
    MotionActivityClassifierConfig config_{};
    StairMotionDetector stair_detector_{};
    StairGeometryEstimator stair_geometry_{};
    VehicleMotionDetector vehicle_detector_{};
    VerticalTransportDetector vertical_transport_{};
    MotionTransitionFsm fsm_{};

    MotionActivityEstimate last_estimate_{};
    float current_horizontal_speed_{0.0F};
    float prev_baro_alt_{0.0F};
    uint64_t prev_baro_timestamp_ns_{0};
    float baro_vertical_rate_{0.0F};
    bool has_baro_datum_{false};  // [M6] đã nhận mẫu baro hợp lệ (baro có thể âm gần mực nước biển)

    // [C1] Pitch nền tự hiệu chuẩn: điện thoại trong túi/tay nghiêng tĩnh 45-65°,
    // ngưỡng pitch tuyệt đối ±0.08 rad vô nghĩa. Baseline = pitch khi đứng yên
    // (EMA), mọi quyết định hướng dùng relative pitch = pitch − baseline.
    double baseline_pitch_rad_{0.0};
    uint32_t baseline_pitch_samples_{0};
    bool has_baseline_pitch_{false};

    // [A5] Phiên thang: landing = khoảng WALKING ngắn (<= 8s) giữa 2 lần vào vế
    // thang. Khi quay lại vế thang trong cửa sổ -> gọi recordLandingStep() hồi tố.
    bool in_stair_session_{false};
    uint64_t last_stair_step_ts_ns_{0};
    bool session_landing_recorded_{false};  // [A5] chống đếm kép landing/phiên
    int last_detector_stair_dir_{0};        // [A6] 0=none, +1 ascent, −1 descent

    // [C1] Cửa sổ p_stair (kích thước/ngưỡng cấu hình qua config_.stair_sustain_*):
    // nhãn STAIRS_* chỉ khi bằng chứng thang DUY TRÌ — p_stair của đi bộ dao động
    // thấp sẽ trả nhãn về WALKING thay vì khóa sai cả lượt đi thành "xuống thang".
    float recent_p_stair_[8]{};
    uint32_t recent_p_stair_idx_{0};
    uint32_t recent_p_stair_count_{0};
    bool last_stair_sustained_{false};  // [A2-fix] cache stair_sustained mỗi frame

    // [C1] Cadence lọc median 3 bước: cadence per-step tức thời nhấp nháy
    // 1.4-2.6 Hz trên đi bộ (đo thực tế) — không lọc thì đường RUNNING phụ
    // nháy theo từng bước và FSM ngoài nhảy WALKING<->RUNNING liên tục.
    float cadence_hist_[3]{0.0F, 0.0F, 0.0F};
    uint32_t cadence_hist_count_{0};
    uint64_t last_step_frame_ts_ns_{0};  // [C1] history chỉ hợp lệ trong 1.5s sau bước cuối
};

} // namespace survival::kinematics
