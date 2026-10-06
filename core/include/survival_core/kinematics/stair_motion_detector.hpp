#pragma once

#include "survival_core/types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace survival::kinematics {

// [C4] Cấu hình StairMotionDetector — mặc định = giá trị đã hiệu chỉnh theo
// audit dữ liệu thật (docs/improvement_report_2026-09-28.md §2-C1c).
struct StairMotionDetectorConfig {
    // p_stair: dải biên độ p2p của THANG (m/s^2): đi bộ túi 23-35 LỚN HƠN thang
    double p_stair_base{0.25};
    double p_lo_ramp{6.0};        // p_lo = clamp((p2p - 3.0)/6.0) — 4.0 làm Z_end D1/D2 tệ (a6d)
    double p_hi_start{16.0};      // p_hi = clamp(1 - (p2p - 16)/10)
    double p_hi_falloff{10.0};
    double p_vh_start{4.0};       // p_vh = clamp((v2h - 4)/4)
    double p_vh_scale{4.0};

    // Bảo vệ flat-walk: delta-Z ≈ 0 -> cap p_stair
    double flat_guard_dz_m{0.04};
    double flat_guard_cap{0.30};

    // Softmax hướng (log-likelihood weights) — skewness là kênh bền nhất
    double w_rise{0.10};
    double w_freefall{0.10};
    double w_p2p{0.10};
    double w_dz{0.20};
    double w_skew{0.50};
    double skew_gain{1.20};       // f5_ascent = 0.5 + skew * gain
    double dz_gain{0.30};         // f4_ascent = 0.5 + (dz/0.25) * gain
    double rise_ascent_s{0.22};   // rise >= 0.22 -> f1_ascent 0.85
    double rise_descent_s{0.16};  // rise <= 0.16 -> f1_ascent 0.15

    // FSM: xác nhận 2 bước cùng chiều, anti-flip 2 bước, thoát lock-on 4 bước p thấp
    uint32_t confirm_steps{2};
    uint32_t anti_flip_steps{2};
    uint32_t low_p_exit_steps{4};

    // [A5] Chiếu nghỉ (landing) — bằng chứng TƯƠNG ĐỐI: bước landing là bước
    // WALKING-like giữa 2 vế, và trên dữ liệu đo, đi bộ túi quần có p2p CAO HƠN
    // thang (23-35 vs 8-19) nên landing = p2p NHẢY LÊN so với median vế thang.
    // (Ngưỡng tuyệt đối cũ p2p<4.8 && v2h<0.85 không bao giờ kích hoạt; bản nới
    // tuyệt đối p2p<10 nhầm cả bước thang biên thấp — đã bỏ.)
    double landing_p2p_jump_ratio{1.75};  // p2p > 1.75 x median vế (chặn spike biên)
    double landing_p2p_abs_min{12.0};     // và p2p > 12 m/s² (walking-like tuyệt đối)
    uint32_t landing_consecutive_steps{4};// liên tiếp >= 4 bước mới khai landing
};

enum class StairActivity : uint8_t {
    kNone = 0,
    kAscent,
    kDescent,
    kPaused,
    kLanding
};

enum class FlightPhase : uint8_t {
    kUnknown = 0,
    kCandidate,
    kConfirmed,
    kLandingTransition
};

struct StairMotionState {
    StairActivity activity{StairActivity::kNone};
    float p_stair{0.0F};
    float p_ascent{0.0F};
    float p_descent{0.0F};
    float p_unknown{1.0F};

    float step_height_m{0.17F};          ///< Chiều cao bậc thang thích ứng (Adaptive Riser)
    float vertical_displacement_m{0.0F}; ///< Tổng độ dịch chuyển tích lũy Z
    float last_step_delta_z_m{0.0F};     ///< [R1] Delta-Z ZUPT của bước gần nhất (bằng chứng độc lập)
    float last_rise_time_s{0.15F};       ///< [C1] Rise-time bước gần nhất (chẩn đoán/audit cực tính)
    float last_skewness{0.0F};           ///< [C1] Bất đối xứng năng lượng bước gần nhất (chẩn đoán)
    float last_v_to_h_ratio{0.0F};       ///< [C1] Tỷ số phương sai đứng/ngang cửa sổ 1s (chẩn đoán)
    float last_peak_to_peak_m_s2{0.0F};  ///< [C1] Biên độ đỉnh-đáy bước gần nhất (chẩn đoán)
    float last_freefall_ms{0.0F};        ///< [C1] Thời gian hụt trọng lực bước gần nhất (chẩn đoán)
    float step_confidence{0.0F};
    float direction_confidence{0.0F};
    float flight_confidence{0.0F};

    uint32_t flight_step_count{0};
    FlightPhase phase{FlightPhase::kUnknown};
};

struct StairDetectorFeatures {
    float rise_time_s{0.0F};
    float peak_accel_m_s2{0.0F};
    float trough_accel_m_s2{0.0F};
    float peak_to_peak_m_s2{0.0F};
    float freefall_dip_m_s2{0.0F};
    float freefall_duration_ms{0.0F};
    float vertical_jerk_m_s3{0.0F};

    float waveform_skewness{0.0F};
    float step_delta_z_m{0.0F};
    float zupt_confidence{0.0F};

    float ascent_likelihood{0.0F};
    float descent_likelihood{0.0F};

    float ascent_score{0.0F};
    float descent_score{0.0F};
    bool is_landing_candidate{false};
};

/**
 * @brief Probabilistic Stair Motion Engine
 * Thực hiện nhận diện chuyển động cầu thang 3D đa pha theo chuẩn suy diễn xác suất
 */
class StairMotionDetector {
public:
    explicit StairMotionDetector(const StairMotionDetectorConfig& config = StairMotionDetectorConfig{}) noexcept;

    /**
     * @brief Xử lý một mẫu gia tốc đứng ở 100 Hz
     * @param vertical_accel_m_s2 Gia tốc thẳng đứng trong hệ quy chiếu Trái Đất (Earth-Frame)
     * @param horizontal_accel_norm Độ lớn gia tốc trên mặt bằng ngang
     * @param dt_s Khoảng thời gian giữa 2 mẫu
     * @param is_step_peak Cờ báo phát hiện đỉnh bước chân
     * @param orientation_confidence Độ tin cậy của ước lượng thái độ ESKF
     */
    [[nodiscard]] StairDetectorFeatures processSample(
        double vertical_accel_m_s2,
        double horizontal_accel_norm,
        double dt_s,
        bool is_step_peak,
        float orientation_confidence = 1.0F
    ) noexcept;

    [[nodiscard]] const StairMotionState& getMotionState() const noexcept {
        return state_;
    }

    void reset() noexcept;

private:
    StairMotionDetectorConfig config_{};

    // Cấu hình buffer chu kỳ bước
    static constexpr size_t kMaxStepBuffer = 128;
    std::array<float, kMaxStepBuffer> step_accel_buf_{};
    size_t step_buf_len_{0};

    // Buffer cửa sổ phương sai tính Stairness (1.0s = 100 samples)
    static constexpr size_t kVarianceWindow = 100;
    std::array<float, kVarianceWindow> window_vert_accel_{};
    std::array<float, kVarianceWindow> window_horiz_accel_{};
    size_t window_idx_{0};
    bool window_filled_{false};

    // Tracking thời gian và cực trị
    double current_trough_val_{0.0};
    double current_peak_val_{0.0};
    double time_since_trough_s_{0.0};
    double time_since_peak_s_{0.0};
    double min_during_step_m_s2_{999.0};
    double freefall_time_s_{0.0};
    bool is_in_rising_phase_{false};

    // ZUPT Step Integrator
    double current_v_z_m_s_{0.0};
    double step_displacement_z_m_{0.0};

    // Bộ đệm P2P cho phát hiện Chiếu nghỉ
    std::array<float, 5> recent_p2p_{};
    size_t history_idx_{0};

    // FSM State Tracking
    StairMotionState state_{};
    float time_since_last_step_s_{0.0F};
    uint32_t consecutive_ascent_steps_{0};
    uint32_t consecutive_descent_steps_{0};
    uint32_t consecutive_low_p_stair_steps_{0};  // [C1] đếm bước p_stair thấp khi kConfirmed (thoát lock-on)

    // Giá trị lưu trữ cache
    float flight_p2p_median_ema_{10.0F};   // [A5] median-EMA p2p của các bước THANG
    uint32_t consecutive_landing_steps_{0}; // [A5] đếm bước walking-like liên tiếp

    // [A6] Đảo hướng TƯƠNG ĐỐI trong phiên thang: skewness tuyệt đối lệch theo
    // vị trí đặt máy nhưng TRONG 1 phiên (lên rồi xuống) độ lệch giữa 2 pha ổn
    // định (đo D1: −0.09 vs −0.29, D2: −0.01 vs −0.10). Cluster hiện hành theo
    // EMA; lệch > ngưỡng K bước liên tiếp -> đảo hướng + re-seed cụm.
    float session_skew_ema_{0.0F};
    uint32_t session_skew_deviant_steps_{0};
    bool session_skew_seeded_{false};
    int session_skew_seed_dir_{0};  // [A6] hướng của cụm lúc seed: +1 ascent, −1 descent
    double stair_exit_timestamp_s_{-100.0}; // [A5] mốc thoát kConfirmed (đếm landing hồi tố)
    bool in_stair_session_{false};          // [A5] đang trong phiên thang (cửa sổ 8s)
    float last_ascent_score_{0.0F};
    float last_descent_score_{0.0F};
    float last_vertical_jerk_m_s3_{0.0F};
    float last_skewness_{0.0F};
    float last_step_delta_z_{0.0F};
    float last_zupt_confidence_{0.0F};
    float last_freefall_ms_{0.0F};
    bool last_is_landing_{false};

    // Các hàm tính toán con
    float computeSkewness(const float* data, size_t len) noexcept;
    void updateFsm(float p_stair, float p_ascent, float p_descent, bool is_step_peak, float dt_s) noexcept;
    void updateAdaptiveRiser(float delta_z, float confidence) noexcept;
};

} // namespace survival::kinematics
