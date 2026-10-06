#pragma once

#include "survival_core/types.hpp"
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace survival::kinematics {

struct StairGeometryConfig {
    float default_riser_m{0.165F};      // [m] Chiều cao bậc thang danh định mặc định
    float min_valid_riser_m{0.120F};    // [m] Chiều cao bậc tối thiểu
    float max_valid_riser_m{0.220F};    // [m] Chiều cao bậc tối đa
    float default_tread_m{0.280F};      // [m] Chiều rộng mặt bậc ngang
    uint32_t min_steps_for_lock{3};     // Số bước ổn định để khóa ước lượng bậc
};

class StairGeometryEstimator {
public:
    explicit StairGeometryEstimator(const StairGeometryConfig& config = StairGeometryConfig{}) noexcept;

    /**
     * @brief Cập nhật một bước cầu thang mới với độ cao vi sai từ áp kế (nếu có)
     * @param is_ascent true nếu leo lên, false nếu đi xuống
     * @param delta_baro_alt_m Độ cao vi sai đo được từ áp kế (0 nếu không có áp kế)
     * @param has_baro true nếu có cảm biến áp kế hợp lệ
     * @param step_interval_s Chu kỳ bước chân [s]
     */
    void recordStairStep(bool is_ascent, double delta_baro_alt_m, bool has_baro, double step_interval_s) noexcept;

    /**
     * @brief Ghi nhận một bước trên chiếu nghỉ (Landing)
     */
    void recordLandingStep() noexcept;

    /**
     * @brief Đặt lại bộ theo dõi cầu thang khi thoát khỏi trạng thái cầu thang
     */
    void reset() noexcept;

    [[nodiscard]] const StairState& getStairState() const noexcept {
        return state_;
    }

    [[nodiscard]] float getEstimatedRiserM() const noexcept {
        return state_.estimated_riser_m;
    }

    [[nodiscard]] float getNominalTreadM() const noexcept {
        return config_.default_tread_m;
    }

private:
    StairGeometryConfig config_{};
    StairState state_{};
    uint32_t consecutive_flight_steps_{0};
    uint32_t valid_baro_delta_count_{0};  // [M7] số delta baro HỢP LỆ trong chặng (mẫu số đúng)
    double flight_baro_accum_m_{0.0};
};

} // namespace survival::kinematics
