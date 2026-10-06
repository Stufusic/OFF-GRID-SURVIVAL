#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/estimators/particle_filter_engine.hpp"
#include "survival_core/kinematics/step_detector.hpp"
#include "survival_core/kinematics/adaptive_parametric_model.hpp"
#include "survival_core/kinematics/stride_scale_estimator.hpp"
#include "survival_core/kinematics/motion_activity_classifier.hpp"
#include "survival_core/kinematics/instantaneous_velocity_estimator.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/integrity/integrity_monitor.hpp"
#include "survival_core/integrity/integrity_guard.hpp"

namespace survival::estimators {

struct PipelineConfig {
    EskfConfig eskf_config{};
    kinematics::AdaptiveModelParams stride_config{};
    ParticleFilterConfig pf_config{};
    double initial_baro_alt_m{1000.0};
    uint32_t random_seed{42};
    // [A3] Phản hồi PF -> PDR có giới hạn (gain 0.02/emit, Mahalanobis ≤ 9,
    // chỉ khi 1 cụm + neff ≥ 30%): PDR hưởng lợi từ DEM/TRN mà không bị kéo nhiễu.
    bool pdr_pf_feedback{true};
    // [A2] Hiệu chuẩn stride theo GNSS (hệ số K kiểu Weiberg): quãng đường GNSS
    // giữa 2 anchor vs quãng đường PDR cùng khoảng -> EMA scale.
    bool gnss_stride_calibration{true};
};

/**
 * @brief Động cơ Hợp nhất Toàn trình Đa Phương thức (Multi-Modal Fusion Pipeline)
 * Tích hợp đa tần số: IMU 100 Hz, Mag 20 Hz, Baro 10 Hz, Step Detector, Motion Activity, Velocity 100Hz, DEM TRN.
 */
class MultiModalFusionPipeline {
public:
    explicit MultiModalFusionPipeline(const PipelineConfig& config = PipelineConfig{}) noexcept;

    /**
     * @brief Khởi tạo pipeline tại vị trí xuất phát kèm nạp bản đồ DEM
     */
    void initialize(const EastingNorthingAlt& initial_pos, const environment::DemMap& dem) noexcept;

    /**
     * @brief Xử lý một khung dữ liệu cảm biến đa tốc độ SensorFrame
     * @return std::optional<NavigationUncertaintyState> Xuất trạng thái bất định mới mỗi khi có cập nhật bước chân
     */
    [[nodiscard]] std::optional<NavigationUncertaintyState> processSensorFrame(const SensorFrame& frame) noexcept;

    [[nodiscard]] const Eskf3D& getEskf() const noexcept {
        return eskf_;
    }

    [[nodiscard]] const ParticleFilterEngine& getParticleFilter() const noexcept {
        return pf_engine_;
    }

    [[nodiscard]] const environment::DemMap& getDemMap() const noexcept {
        return dem_;
    }

    [[nodiscard]] const kinematics::MotionActivityClassifier& getActivityClassifier() const noexcept {
        return activity_classifier_;
    }

    [[nodiscard]] const kinematics::InstantaneousVelocityEstimator& getVelocityEstimator() const noexcept {
        return velocity_estimator_;
    }

    [[nodiscard]] double getLatestNis() const noexcept {
        return latest_nis_;
    }

    [[nodiscard]] double getFilteredBaroAltitude() const noexcept {
        return filtered_baro_alt_m_;
    }

    [[nodiscard]] EastingNorthingAlt getDeadReckoningPosition() const noexcept {
        return current_pdr_pos_;
    }

    /// Hiệp phương sai PDR 2D [var_e, cov_en, cov_en, var_n] [m^2] — cấp đúng đơn vị cho NEES (audit H3)
    [[nodiscard]] std::array<double, 4> getDeadReckoningCovariance() const noexcept {
        return current_pdr_cov_;
    }

    [[nodiscard]] size_t getTotalSteps() const noexcept {
        return total_steps_;
    }

    /**
     * @brief Dời điểm neo (GPS Cutoff Handoff) bằng phép tịnh tiến phân phối hạt delta
     * Bảo toàn nguyên vẹn 100% hình thái phân phối xác suất, lịch sử bước chân và bộ lọc khí áp
     */
    void updateAnchorPosition(const EastingNorthingAlt& new_anchor, double anchor_std_m) noexcept;

    [[nodiscard]] const CanonicalNavigationState& getLatestCanonicalState() const noexcept {
        return latest_canonical_state_;
    }

    [[nodiscard]] const IntegrityStatus& getLatestIntegrityStatus() const noexcept {
        return latest_integrity_status_;
    }

    [[nodiscard]] const integrity::IntegrityGuard& getIntegrityGuard() const noexcept {
        return integrity_guard_;
    }

    /// [A2] Hệ số hiệu chuẩn sải bước đang dùng (1.0 = chưa hiệu chỉnh)
    [[nodiscard]] double getStrideScale() const noexcept { return stride_scale_; }

    [[nodiscard]] double getEffectiveParticleCount() const noexcept {
        return pf_engine_.getEffectiveParticleCount();
    }

    void setTargetParticleCount(size_t count) noexcept {
        pf_engine_.setTargetParticleCount(count);
    }

    /// [B3] Từ sai từ kế (rad, Đông-dương)
    void setMagDeclinationRad(double declination_rad) noexcept {
        eskf_.setMagDeclinationRad(declination_rad);
    }

    /// [B2] Bias áp kế QNH đang ước lượng [m] (baro = DEM + bias)
    [[nodiscard]] double getBaroBiasM() const noexcept {
        return baro_bias_m_;
    }

    void resetEstimatorSeed(uint32_t seed) noexcept {
        pf_engine_.setRandomSeed(seed);
    }

private:
    PipelineConfig config_{};
    Eskf3D eskf_{};

    // [A2] Hiệu chuẩn stride theo GNSS
    kinematics::StrideScaleEstimator stride_scale_est_{};
    double stride_scale_{1.0};            // hệ số K hiện hành (mirror của estimator)
    double pdr_dist_since_anchor_m_{0.0}; // quãng đường PDR tích lũy giữa 2 anchor
    bool has_prev_anchor_{false};
    EastingNorthingAlt prev_anchor_{};
    kinematics::StepDetector step_detector_{};
    kinematics::AdaptiveParametricModel stride_model_{};
    kinematics::MotionActivityClassifier activity_classifier_{};
    kinematics::InstantaneousVelocityEstimator velocity_estimator_{};
    ParticleFilterEngine pf_engine_{};
    environment::DemMap dem_{};
    integrity::IntegrityMonitor integrity_monitor_{};
    integrity::IntegrityGuard integrity_guard_{};

    uint64_t last_timestamp_ns_{0};
    uint64_t last_output_timestamp_ns_{0};
    double filtered_baro_alt_m_{1000.0};
    double prev_baro_rate_m_{1000.0};        // [M1] mẫu baro trước (đúng nhịp 10 Hz)
    uint64_t last_baro_ts_ns_{0};
    double last_baro_rate_m_s_{0.0};
    double baro_bias_m_{0.0};                // [B2] bias ap ke QNH duoc uoc luong [m]         // [M1] toc do doc hold giua 2 mau baro            // [M1] timestamp mẫu baro trước
    double last_alt_sample_m_{1000.0};
    double last_adaptive_stride_m_{0.75};    // [M2] sải bước thích ứng cuối (cho velocity)
    double last_pdr_slope_rad_{0.0};
    double latest_nis_{0.0};
    size_t total_steps_{0};

    EastingNorthingAlt current_pdr_pos_{};
    std::array<double, 4> current_pdr_cov_{1.0, 0.0, 0.0, 1.0};
    bool is_initialized_{false};
    bool has_initial_baro_{false};

    CanonicalNavigationState latest_canonical_state_{};
    IntegrityStatus latest_integrity_status_{};
};

} // namespace survival::estimators
