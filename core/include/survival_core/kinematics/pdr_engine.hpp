#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/estimators/eskf_3d.hpp"
#include "survival_core/kinematics/step_detector.hpp"
#include "survival_core/kinematics/adaptive_parametric_model.hpp"

namespace survival::kinematics {

class PdrEngine {
public:
    explicit PdrEngine(std::unique_ptr<IStepLengthModel> stride_model = nullptr,
                       const estimators::EskfConfig& eskf_config = estimators::EskfConfig{},
                       const StepDetectorConfig& detector_config = StepDetectorConfig{}) noexcept;

    /**
     * @brief Khởi tạo lại vị trí và bộ ước lượng
     */
    void reset(const EastingNorthingAlt& origin = EastingNorthingAlt{0.0, 0.0, 1000.0}) noexcept;

    /**
     * @brief Xử lý 1 khung dữ liệu cảm biến (100 Hz)
     * @return std::optional<StepEvent> chứa thông tin bước chân nếu bước chân vừa được phát hiện tại khung này
     */
    [[nodiscard]] std::optional<StepEvent> processFrame(const SensorFrame& frame, double dt_s) noexcept;

    [[nodiscard]] const EastingNorthingAlt& getCurrentPosition() const noexcept {
        return current_position_;
    }

    [[nodiscard]] std::array<double, 4> getPositionCovariance2D() const noexcept {
        return {var_e_, cov_en_, cov_en_, var_n_};
    }

    [[nodiscard]] estimators::AttitudeEstimate getAttitudeEstimate() const noexcept {
        return eskf_.getAttitudeEstimate();
    }

    [[nodiscard]] size_t getTotalSteps() const noexcept {
        return step_detector_.getTotalStepCount();
    }

    [[nodiscard]] const estimators::Eskf3D& getEskf() const noexcept {
        return eskf_;
    }

private:
    estimators::Eskf3D eskf_;
    StepDetector step_detector_;
    std::unique_ptr<IStepLengthModel> stride_model_;

    EastingNorthingAlt current_position_{0.0, 0.0, 1000.0};
    double var_e_{0.01};
    double var_n_{0.01};
    double cov_en_{0.0};

    double current_baro_alt_m_{1000.0};
    double last_step_alt_m_{1000.0};
    bool has_initial_baro_{false};
};

} // namespace survival::kinematics
