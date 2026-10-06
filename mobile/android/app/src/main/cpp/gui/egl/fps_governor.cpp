#include "fps_governor.hpp"
#include <cmath>

namespace survival::gui {

void FpsGovernor::updateMotionState(bool is_moving, double heading_delta_rad, double battery_soc, bool is_screen_on) noexcept {
    if (!is_screen_on) {
        state_ = RenderActivityState::kScreenOff;
        frame_interval_ns_ = UINT64_MAX; // 0 FPS
        return;
    }

    if (battery_soc <= 5.0) {
        state_ = RenderActivityState::kBatteryCritical;
        frame_interval_ns_ = 10000000000ULL; // 0.1 FPS (10 giây/khung)
        return;
    }

    if (is_moving || std::abs(heading_delta_rad) > 0.017) { // > 1.0 độ
        state_ = RenderActivityState::kMovingOrRotating;
        frame_interval_ns_ = 16666667ULL; // 60 FPS (16.6 ms)
    } else {
        state_ = RenderActivityState::kStationary;
        frame_interval_ns_ = 1000000000ULL; // 1 FPS (1.0 s)
    }
}

bool FpsGovernor::shouldRenderFrame(uint64_t current_time_ns) noexcept {
    if (state_ == RenderActivityState::kScreenOff) {
        return false;
    }

    if (last_render_time_ns_ == 0 || (current_time_ns >= last_render_time_ns_ + frame_interval_ns_)) {
        last_render_time_ns_ = current_time_ns;
        return true;
    }

    return false;
}

double FpsGovernor::getTargetFps() const noexcept {
    switch (state_) {
        case RenderActivityState::kScreenOff: return 0.0;
        case RenderActivityState::kBatteryCritical: return 0.1;
        case RenderActivityState::kStationary: return 1.0;
        case RenderActivityState::kMovingOrRotating: return 60.0;
    }
    return 1.0;
}

} // namespace survival::gui
