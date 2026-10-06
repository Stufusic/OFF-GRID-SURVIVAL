#pragma once

#include <cstdint>
#include <array>
#include <span>
#include <cmath>
#include "survival_core/data_contracts.hpp"

namespace survival::integrity {

struct IntegrityTelemetry {
    uint64_t nan_inf_detected{0};
    uint64_t accel_out_of_bounds{0};
    uint64_t mag_out_of_bounds{0};
    uint64_t baro_out_of_bounds{0};
    uint64_t psd_covariance_repairs{0};
    uint64_t cholesky_failures{0};
};

class IntegrityGuard {
public:
    IntegrityGuard() = default;

    /**
     * @brief Kiểm tra và làm sạch gói SensorFrame đầu vào
     * @return true nếu dữ liệu hợp lệ hoặc đã được sửa an toàn; false nếu phát hiện lỗi nghiêm trọng
     */
    [[nodiscard]] bool sanitizeSensorFrame(SensorFrame& frame) noexcept;

    /**
     * @brief Kiểm tra tính hợp lệ của sự kiện bước chân PDR
     */
    [[nodiscard]] bool sanitizeStepEvent(StepEvent& step) noexcept;

    /**
     * @brief Ép đối xứng và bảo toàn tính xác định dương (PSD) cho ma trận hiệp phương sai 2x2
     * @param cov_en Tham số đầu ra mảng 2x2 [var_e, cov_en, cov_en, var_n]
     */
    [[nodiscard]] bool enforcePsd2x2(double& var_e, double& var_n, double& cov_en, double min_diag = 1e-8) noexcept;

    /**
     * @brief Ép đối xứng và bảo toàn tính xác định dương (PSD) cho ma trận hiệp phương sai 3x3
     * @param p_flat Mảng 9 phần tử lưu theo dòng (Row-Major)
     */
    [[nodiscard]] bool enforcePsd3x3(std::array<double, 9>& p_flat, double min_eigenvalue = 1e-8) noexcept;

    /**
     * @brief Lấy báo cáo thống kê Telemetry
     */
    [[nodiscard]] const IntegrityTelemetry& getTelemetry() const noexcept {
        return telemetry_;
    }

    void resetTelemetry() noexcept {
        telemetry_ = {};
    }

private:
    IntegrityTelemetry telemetry_{};
};

} // namespace survival::integrity
