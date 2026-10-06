#pragma once

#include <cstdint>

namespace survival::gui {

enum class RenderActivityState : uint8_t {
    kStationary = 0,    // Đứng yên / Xem dữ liệu -> 1 FPS
    kMovingOrRotating = 1, // Đang di chuyển / Xoay máy -> 30-60 FPS
    kBatteryCritical = 2,  // Pin 2-5% -> 0.1 FPS (10s/frame)
    kScreenOff = 3         // Màn hình tắt -> 0 FPS
};

class FpsGovernor {
public:
    FpsGovernor() noexcept = default;

    /**
     * @brief Đánh giá xem có nên render khung hình tại thời điểm hiện tại
     */
    [[nodiscard]] bool shouldRenderFrame(uint64_t current_time_ns) noexcept;

    /**
     * @brief Cập nhật trạng thái chuyển động của người dùng
     */
    void updateMotionState(bool is_moving, double heading_delta_rad, double battery_soc, bool is_screen_on) noexcept;

    [[nodiscard]] RenderActivityState getActivityState() const noexcept {
        return state_;
    }

    [[nodiscard]] double getTargetFps() const noexcept;

private:
    RenderActivityState state_{RenderActivityState::kStationary};
    uint64_t last_render_time_ns_{0};
    uint64_t frame_interval_ns_{1000000000ULL}; // Mặc định 1 FPS = 1.000.000.000 ns
};

} // namespace survival::gui
