#include "survival_core/integrity/integrity_guard.hpp"
#include "survival_core/math/angle_utils.hpp"
#include <Eigen/Dense>
#include <algorithm>

namespace survival::integrity {

namespace {

inline bool isInvalid(float v) noexcept {
    return std::isnan(v) || std::isinf(v);
}

inline bool isInvalid(double v) noexcept {
    return std::isnan(v) || std::isinf(v);
}

} // namespace

bool IntegrityGuard::sanitizeSensorFrame(SensorFrame& frame) noexcept {
    bool has_error = false;

    // 1. Kiểm tra NaN / Inf trên gia tốc và con quay hồi chuyển
    for (size_t i = 0; i < 3; ++i) {
        if (isInvalid(frame.accel_m_s2[i]) || isInvalid(frame.gyro_rad_s[i])) {
            telemetry_.nan_inf_detected++;
            has_error = true;
            frame.has_imu = false;
            frame.accel_m_s2[i] = (i == 2) ? static_cast<float>(kGravityStandard) : 0.0F;
            frame.gyro_rad_s[i] = 0.0F;
        }
    }

    // 2. Giới hạn vật lý gia tốc |a| <= 50 m/s^2
    const float accel_norm_sq = frame.accel_m_s2[0] * frame.accel_m_s2[0] +
                                frame.accel_m_s2[1] * frame.accel_m_s2[1] +
                                frame.accel_m_s2[2] * frame.accel_m_s2[2];
    if (accel_norm_sq > 2500.0F) { // 50^2
        telemetry_.accel_out_of_bounds++;
        has_error = true;
        frame.has_imu = false;
    }

    // 3. Kiểm tra Từ kế
    for (size_t i = 0; i < 3; ++i) {
        if (isInvalid(frame.mag_uT[i])) {
            telemetry_.nan_inf_detected++;
            has_error = true;
            frame.has_mag = false;
            frame.mag_uT[i] = 0.0F;
        }
    }
    const float mag_norm_sq = frame.mag_uT[0] * frame.mag_uT[0] +
                              frame.mag_uT[1] * frame.mag_uT[1] +
                              frame.mag_uT[2] * frame.mag_uT[2];
    if (mag_norm_sq > 40000.0F) { // 200 uT^2
        telemetry_.mag_out_of_bounds++;
        frame.has_mag = false;
    }

    // 4. Kiểm tra Áp kế
    if (isInvalid(frame.pressure_hpa) || frame.pressure_hpa < 300.0F || frame.pressure_hpa > 1200.0F) {
        telemetry_.baro_out_of_bounds++;
        has_error = true;
        frame.has_baro = false;
        frame.pressure_hpa = static_cast<float>(kStandardPressureHpa);
    }

    // 5. Kiểm tra Optical Sun
    if (frame.optical_sun_azimuth_rad >= 0.0F) {
        if (isInvalid(frame.optical_sun_azimuth_rad) || isInvalid(frame.optical_confidence)) {
            telemetry_.nan_inf_detected++;
            frame.optical_sun_azimuth_rad = -1.0F;
            frame.optical_confidence = 0.0F;
            frame.has_optical_sun = false;
        } else {
            frame.optical_sun_azimuth_rad = math::normalizeHeadingRad(frame.optical_sun_azimuth_rad);
            frame.optical_confidence = std::clamp(frame.optical_confidence, 0.0F, 1.0F);
            frame.has_optical_sun = true;
        }
    } else {
        frame.has_optical_sun = false;
    }

    return !has_error;
}

bool IntegrityGuard::sanitizeStepEvent(StepEvent& step) noexcept {
    bool ok = true;

    if (isInvalid(step.stride_mean_m) || step.stride_mean_m < 0.05 || step.stride_mean_m > 3.0) {
        telemetry_.nan_inf_detected++;
        step.stride_mean_m = 0.75;
        ok = false;
    }

    if (isInvalid(step.stride_variance_m2) || step.stride_variance_m2 <= 0.0) {
        step.stride_variance_m2 = 0.01;
        ok = false;
    }

    if (isInvalid(step.heading_rad)) {
        telemetry_.nan_inf_detected++;
        step.heading_rad = 0.0;
        ok = false;
    } else {
        step.heading_rad = math::normalizeHeadingRad(step.heading_rad);
    }

    if (isInvalid(step.heading_variance_rad2) || step.heading_variance_rad2 <= 0.0) {
        step.heading_variance_rad2 = 0.01;
        ok = false;
    }

    if (isInvalid(step.confidence)) {
        step.confidence = 0.5;
        ok = false;
    } else {
        step.confidence = std::clamp(step.confidence, 0.0, 1.0);
    }

    return ok;
}

bool IntegrityGuard::enforcePsd2x2(double& var_e, double& var_n, double& cov_en, double min_diag) noexcept {
    bool repaired = false;

    if (isInvalid(var_e) || var_e < min_diag) {
        var_e = min_diag;
        repaired = true;
    }
    if (isInvalid(var_n) || var_n < min_diag) {
        var_n = min_diag;
        repaired = true;
    }
    if (isInvalid(cov_en)) {
        cov_en = 0.0;
        repaired = true;
    }

    // Định thức det = var_e * var_n - cov_en^2 >= 0
    const double max_cov = std::sqrt(var_e * var_n) * 0.9999; // Giữ margin an toàn
    if (std::abs(cov_en) > max_cov) {
        cov_en = (cov_en > 0.0) ? max_cov : -max_cov;
        repaired = true;
    }

    if (repaired) {
        telemetry_.psd_covariance_repairs++;
    }

    return repaired;
}

bool IntegrityGuard::enforcePsd3x3(std::array<double, 9>& p_flat, double min_eigenvalue) noexcept {
    bool repaired = false;

    // 1. Ánh xạ Eigen Map
    Eigen::Matrix3d P;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double val = p_flat[static_cast<size_t>(r * 3 + c)];
            if (isInvalid(val)) {
                val = (r == c) ? 1.0 : 0.0;
                repaired = true;
                telemetry_.nan_inf_detected++;
            }
            P(r, c) = val;
        }
    }

    // 2. Ép đối xứng: P = 0.5 * (P + P^T)
    P = 0.5 * (P + P.transpose());

    // 3. Kiểm tra Cholesky Decomposition
    Eigen::LLT<Eigen::Matrix3d> llt(P);
    if (llt.info() != Eigen::Success) {
        telemetry_.cholesky_failures++;
        repaired = true;

        // Phân tích Trị riêng SelfAdjoint để sửa trị riêng âm
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
        if (es.info() == Eigen::Success) {
            Eigen::Vector3d eigenvalues = es.eigenvalues();
            for (int i = 0; i < 3; ++i) {
                if (eigenvalues(i) < min_eigenvalue) {
                    eigenvalues(i) = min_eigenvalue;
                }
            }
            P = es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();
            P = 0.5 * (P + P.transpose());
            telemetry_.psd_covariance_repairs++;
        } else {
            // Trường hợp xấu nhất: Reset về ma trận đường chéo
            P = Eigen::Matrix3d::Identity() * min_eigenvalue;
            telemetry_.psd_covariance_repairs++;
        }
    }

    // 4. Ghi ngược lại p_flat
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            p_flat[static_cast<size_t>(r * 3 + c)] = P(r, c);
        }
    }

    return repaired;
}

} // namespace survival::integrity
