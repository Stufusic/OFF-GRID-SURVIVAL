#include "survival_core/integrity/integrity_monitor.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace survival::integrity {

namespace {

inline bool isInvalid(double v) noexcept {
    return std::isnan(v) || std::isinf(v);
}

} // namespace

bool IntegrityMonitor::checkCovariance2D(double var_e, double var_n, double cov_en, double min_eigen) noexcept {
    if (isInvalid(var_e) || isInvalid(var_n) || isInvalid(cov_en)) {
        return false;
    }
    if (var_e < min_eigen || var_n < min_eigen) {
        return false;
    }
    // Định thức det = var_e * var_n - cov_en^2 >= min_eigen^2
    const double det = var_e * var_n - cov_en * cov_en;
    return det >= (min_eigen * min_eigen);
}

bool IntegrityMonitor::checkCovariance3x3(const std::array<double, 9>& cov_3x3, double min_eigen) noexcept {
    Eigen::Matrix3d P;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const double val = cov_3x3[static_cast<size_t>(r * 3 + c)];
            if (isInvalid(val)) {
                return false;
            }
            P(r, c) = val;
        }
    }

    // 1. Kiểm tra đối xứng
    const double asymm_norm = (P - P.transpose()).norm();
    if (asymm_norm > 1e-4) {
        return false;
    }

    // 2. Kiểm tra xác định dương (Cholesky)
    Eigen::LLT<Eigen::Matrix3d> llt(P);
    if (llt.info() != Eigen::Success) {
        return false;
    }

    // 3. Kiểm tra trị riêng tối thiểu
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
    if (es.info() != Eigen::Success || es.eigenvalues().minCoeff() < min_eigen) {
        return false;
    }

    return true;
}

IntegrityStatus IntegrityMonitor::auditState(const CanonicalNavigationState& state, uint64_t current_time_ns) noexcept {
    IntegrityStatus status{};
    status.is_healthy = true;
    status.consistency_score = 1.0;
    status.fault_flags = kFaultNone;

    // 1. [INVARIANT 10]: Kiểm tra NaN / Inf trên toàn bộ các trường
    if (isInvalid(state.position_enu.easting_m) || isInvalid(state.position_enu.northing_m) || isInvalid(state.position_enu.altitude_m) ||
        isInvalid(state.position_wgs84.latitude_deg) || isInvalid(state.position_wgs84.longitude_deg) ||
        isInvalid(state.velocity_m_s.x) || isInvalid(state.velocity_m_s.y) || isInvalid(state.velocity_m_s.z) ||
        isInvalid(state.attitude_q.w) || isInvalid(state.attitude_q.x) || isInvalid(state.attitude_q.y) || isInvalid(state.attitude_q.z) ||
        isInvalid(state.global_navigation_score)) {
        
        status.is_healthy = false;
        status.position_valid = false;
        status.attitude_valid = false;
        status.velocity_valid = false;
        status.covariance_valid = false;
        status.fault_flags |= kFaultNanInfDetected;
        status.consistency_score = 0.0;
        return status;
    }

    // 2. Kiểm tra tính hợp lý của Vận tốc và Gia tốc (Kinematic Plausibility Layer)
    const double v_norm = state.velocity_m_s.norm();
    const bool is_vehicle = (state.motion_activity.current_activity == MotionActivity::kVehicleWheeled);
    const double max_v = is_vehicle ? 60.0 : kMaxPlausibleVelocity;
    const double max_a = is_vehicle ? 12.0 : 6.0; // [m/s^2] Gia tốc vật lý tối đa

    if (prev_timestamp_ns_ > 0 && state.timestamp_ns > prev_timestamp_ns_) {
        const double dt_s = static_cast<double>(state.timestamp_ns - prev_timestamp_ns_) * 1e-9;
        if (dt_s > 1e-4 && dt_s < 2.0) {
            const double apparent_accel = std::abs(v_norm - prev_v_norm_) / dt_s;
            if (apparent_accel > max_a && v_norm > 2.0) {
                status.velocity_valid = false;
                status.fault_flags |= kFaultVelocityExcess;
                status.consistency_score *= 0.7;
            }
        }
    }
    prev_v_norm_ = v_norm;
    prev_timestamp_ns_ = state.timestamp_ns;

    if (v_norm > max_v) {
        status.velocity_valid = false;
        status.fault_flags |= kFaultVelocityExcess;
        status.consistency_score *= 0.5;
    }

    // 3. [INVARIANT 8]: Kiểm tra tính hợp lệ của Ma trận Hiệp phương sai
    const bool cov2d_ok = checkCovariance2D(
        state.position_cov_2d[0],
        state.position_cov_2d[3],
        state.position_cov_2d[1]
    );
    if (!cov2d_ok) {
        status.covariance_valid = false;
        status.fault_flags |= kFaultCovarianceNonPsd;
        status.consistency_score *= 0.3;
    }

    // 4. [INVARIANT 7 - No False Confidence]: Điểm tin cậy không được vượt quá chất lượng cảm biến
    const float min_sensor_health = std::min({
        state.sensor_health.imu,
        state.sensor_health.magnetometer,
        state.sensor_health.barometer
    });
    if (state.global_navigation_score > (static_cast<double>(min_sensor_health) + 0.35) && min_sensor_health < 0.4F) {
        status.fault_flags |= kFaultFalseConfidence;
        status.consistency_score *= 0.7;
    }

    // 5. [INVARIANT 11 - Estimator Freshness Watchdog]
    if (current_time_ns > state.timestamp_ns && (current_time_ns - state.timestamp_ns) > kMaxStaleDurationNs) {
        status.fault_flags |= kFaultEstimatorStale;
        status.consistency_score *= 0.2;
        status.is_healthy = false;
    }

    if (status.fault_flags != kFaultNone) {
        status.is_healthy = false;
    }

    return status;
}

} // namespace survival::integrity
