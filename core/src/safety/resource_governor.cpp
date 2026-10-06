#include "survival_core/safety/resource_governor.hpp"
#include <algorithm>

namespace survival::safety {

GovernorState ResourceGovernor::updateBatterySoc(
    double soc_percent,
    MissionState mission_state,
    NavigationMode nav_mode) noexcept {

    last_soc_ = std::clamp(soc_percent, 0.0, 100.0);

    const bool is_emergency = (mission_state == MissionState::kRescueRequired ||
                               mission_state == MissionState::kDistressLocked ||
                               nav_mode == NavigationMode::kLost ||
                               nav_mode == NavigationMode::kEmergency);

    // [M10] Hysteresis 3%: XUỐNG cấp ngay theo ngưỡng cũ (20/5/2% — pin xụt nhanh
    // được phản ứng tức thì); LÊN cấp phải vượt X+3 (23/8/5%) — trước đây cascade
    // thuần 2 chiều khiến mode nhảy liên tục khi pin dao động ±1-2% dưới tải.
    PowerMode desired;
    if (last_soc_ <= 2.0) {
        desired = PowerMode::kDisplayOnly;
    } else if (last_soc_ <= 5.0) {
        desired = PowerMode::kCriticalSnapshot;
    } else if (last_soc_ <= 20.0) {
        desired = PowerMode::kEnergyConservative;
    } else {
        desired = PowerMode::kBalanced;
    }

    if (desired != current_mode_) {
        const bool going_down = static_cast<uint8_t>(desired) > static_cast<uint8_t>(current_mode_);
        if (going_down) {
            current_mode_ = desired;
        } else {
            const bool can_go_up =
                (current_mode_ == PowerMode::kDisplayOnly && last_soc_ >= 5.0) ||
                (current_mode_ == PowerMode::kCriticalSnapshot && last_soc_ >= 8.0) ||
                (current_mode_ == PowerMode::kEnergyConservative && last_soc_ >= 23.0);
            if (can_go_up) {
                current_mode_ = desired;
            }
        }
    }

    GovernorState state{};
    state.is_emergency_overridden = is_emergency;

    switch (current_mode_) {
        case PowerMode::kBalanced:
            state.power_mode = PowerMode::kBalanced;
            state.target_particle_count = 2000;
            state.fast_loop_rate_hz = 100.0;
            state.pf_update_interval_steps = 1.0;
            state.allow_camera_sun = true;
            state.allow_sensors = true;
            state.should_trigger_disk_flush = false;
            break;

        case PowerMode::kEnergyConservative:
            state.power_mode = PowerMode::kEnergyConservative;
            state.target_particle_count = is_emergency ? 800 : 500;
            state.fast_loop_rate_hz = is_emergency ? 100.0 : 50.0;
            state.pf_update_interval_steps = is_emergency ? 1.0 : 2.0;
            state.allow_camera_sun = false;
            state.allow_sensors = true;
            state.should_trigger_disk_flush = false;
            break;

        case PowerMode::kCriticalSnapshot:
            state.power_mode = PowerMode::kCriticalSnapshot;
            state.target_particle_count = is_emergency ? 400 : 100;
            state.fast_loop_rate_hz = is_emergency ? 50.0 : 20.0;
            state.pf_update_interval_steps = is_emergency ? 2.0 : 5.0;
            state.allow_camera_sun = false;
            state.allow_sensors = true;
            state.should_trigger_disk_flush = true;
            break;

        case PowerMode::kDisplayOnly:
            state.power_mode = PowerMode::kDisplayOnly;
            state.target_particle_count = is_emergency ? 200 : 0;
            state.fast_loop_rate_hz = is_emergency ? 20.0 : 0.0;
            state.pf_update_interval_steps = is_emergency ? 5.0 : 0.0;
            state.allow_camera_sun = false;
            state.allow_sensors = is_emergency;
            state.should_trigger_disk_flush = true;
            break;
    }

    return state;
}

} // namespace survival::safety
