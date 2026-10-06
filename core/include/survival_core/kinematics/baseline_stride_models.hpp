#pragma once

#include "survival_core/kinematics/i_step_length_model.hpp"

namespace survival::kinematics {

/**
 * @brief Mô hình kinh điển Weinberg (2002): L = K * (a_max - a_min)^(1/4)
 */
class WeinbergModel : public IStepLengthModel {
public:
    explicit WeinbergModel(double k = 0.45) noexcept : k_(k) {}
    [[nodiscard]] StrideEstimate estimateStride(const StepInputFeatures& features) const noexcept override;
private:
    double k_{0.45};
};

/**
 * @brief Mô hình kinh điển Kim (2004): L = K * (RMS_a)^(1/3)
 */
class KimModel : public IStepLengthModel {
public:
    explicit KimModel(double k = 0.70) noexcept : k_(k) {}
    [[nodiscard]] StrideEstimate estimateStride(const StepInputFeatures& features) const noexcept override;
private:
    double k_{0.70};
};

/**
 * @brief Mô hình kinh điển Scarlett (2005): L = K * ((a_rms - a_min) / (a_max - a_min))
 */
class ScarlettModel : public IStepLengthModel {
public:
    explicit ScarlettModel(double k = 0.65) noexcept : k_(k) {}
    [[nodiscard]] StrideEstimate estimateStride(const StepInputFeatures& features) const noexcept override;
private:
    double k_{0.65};
};

} // namespace survival::kinematics
