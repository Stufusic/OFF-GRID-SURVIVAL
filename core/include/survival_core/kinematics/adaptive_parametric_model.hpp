#pragma once

#include "survival_core/kinematics/i_step_length_model.hpp"

namespace survival::kinematics {

struct AdaptiveModelParams {
    double alpha{0.55};         // Trọng số nhịp bước chân
    double beta{0.45};          // Trọng số cường độ gia tốc thẳng đứng
    double f0_hz{1.8};          // Nhịp bước chuẩn [Hz]
    double a0_m_s2{9.80665};    // Gia tốc chuẩn [m/s^2]
    // [C3c] 0.04 -> 0.12 m: NEES Pure PDR đo MC = 357 (rvcore.md ~320) — covariance
    //低估 ~18x hệ thống; σ sải thực 0.10-0.20 m trên dữ liệu thật, 0.04 là quá tin.
    double sigma0_m{0.12};      // Phương sai bước chân chuẩn [m]
    double gamma1{0.50};        // Hệ số nhạy với độ dốc
    double gamma2{0.20};        // Hệ số nhạy với độ nhiễu chu kỳ bước
    double tau0_s{0.555};       // Chu kỳ bước chuẩn (1 / 1.8 Hz)
};

class AdaptiveParametricModel : public IStepLengthModel {
public:
    explicit AdaptiveParametricModel(const AdaptiveModelParams& params = AdaptiveModelParams{}) noexcept;

    [[nodiscard]] StrideEstimate estimateStride(const StepInputFeatures& features) const noexcept override;

    /**
     * @brief Hàm suy giảm sải bước phi đối xứng theo cơ sinh học
     */
    [[nodiscard]] static double computeBiomechanicSlopeFactor(double slope_rad) noexcept;

private:
    AdaptiveModelParams params_{};
};

} // namespace survival::kinematics
