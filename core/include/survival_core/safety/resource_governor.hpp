#pragma once

#include "survival_core/types.hpp"
#include "survival_core/estimators/particle_types.hpp"

namespace survival::safety {

struct GovernorState {
    PowerMode power_mode{PowerMode::kBalanced};
    size_t target_particle_count{2000};
    double fast_loop_rate_hz{100.0};
    double pf_update_interval_steps{1.0};
    bool allow_camera_sun{true};
    bool allow_sensors{true};
    bool should_trigger_disk_flush{false};
    bool is_emergency_overridden{false};
};

class ResourceGovernor {
public:
    ResourceGovernor() noexcept = default;

    /**
     * @brief Cập nhật trạng thái phần trăm pin (SoC: State of Charge [0.0 - 100.0])
     */
    GovernorState updateBatterySoc(
        double soc_percent,
        MissionState mission_state = MissionState::kNormal,
        NavigationMode nav_mode = NavigationMode::kGpsAided) noexcept;

    [[nodiscard]] PowerMode getCurrentMode() const noexcept {
        return current_mode_;
    }

    [[nodiscard]] double getLastSoc() const noexcept {
        return last_soc_;
    }

private:
    PowerMode current_mode_{PowerMode::kBalanced};
    double last_soc_{100.0};
};

} // namespace survival::safety
