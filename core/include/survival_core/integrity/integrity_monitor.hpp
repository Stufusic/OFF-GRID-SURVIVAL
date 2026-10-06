#pragma once

#include "survival_core/types.hpp"
#include <array>

namespace survival::integrity {

enum FaultCode : uint32_t {
    kFaultNone = 0,
    kFaultVelocityExcess = 1 << 0,
    kFaultAccelerationExcess = 1 << 1,
    kFaultCovarianceAsymmetry = 1 << 2,
    kFaultCovarianceNonPsd = 1 << 3,
    kFaultCovarianceIllConditioned = 1 << 4,
    kFaultNanInfDetected = 1 << 5,
    kFaultEstimatorStale = 1 << 6,
    kFaultFalseConfidence = 1 << 7,
    kFaultEstimatorDegraded = 1 << 8,
    kFaultSensorDead = 1 << 9
};

/**
 * @brief Bộ giám sát toàn vẹn độc lập (Independent Passive Integrity Monitor)
 * Không bao giờ can thiệp sửa State, chỉ kiểm tra và trả về báo cáo chất lượng
 */
class IntegrityMonitor {
public:
    IntegrityMonitor() noexcept = default;

    /**
     * @brief Đánh giá toàn vẹn trạng thái Navigation State
     */
    [[nodiscard]] IntegrityStatus auditState(
        const CanonicalNavigationState& state,
        uint64_t current_time_ns) noexcept;

    /**
     * @brief Kiểm tra tính hợp lệ hiệp phương sai 2D
     */
    [[nodiscard]] static bool checkCovariance2D(
        double var_e, double var_n, double cov_en, double min_eigen = 1e-8) noexcept;

    /**
     * @brief Kiểm tra tính hợp lệ ma trận hiệp phương sai 3x3
     */
    [[nodiscard]] static bool checkCovariance3x3(
        const std::array<double, 9>& cov_3x3, double min_eigen = 1e-8) noexcept;

private:
    static constexpr double kMaxPlausibleVelocity = 15.0; // [m/s] Vận tốc người đi bộ/chạy tối đa
    static constexpr double kMaxPlausibleAcceleration = 35.0; // [m/s^2] Gia tốc vật lý tối đa
    static constexpr uint64_t kMaxStaleDurationNs = 1500000000ULL; // 1.5s timeout

    double prev_v_norm_{0.0};
    uint64_t prev_timestamp_ns_{0};
};

} // namespace survival::integrity
