#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <vector>
#include "survival_core/data_contracts.hpp"
#include "survival_harness/synthetic_generator.hpp"

namespace survival::harness {

using SensorFrameCallback = std::function<void(const SensorFrame&)>;
using StepEventCallback = std::function<void(const StepEvent&)>;

class DataPlayer {
public:
    DataPlayer() = default;

    /**
     * @brief Nạp dataset vào trình phát lại
     */
    void loadDataset(const SyntheticDataset& dataset) noexcept;

    /**
     * @brief Đăng ký callback nhận SensorFrame
     */
    void setSensorCallback(SensorFrameCallback callback) noexcept {
        sensor_callback_ = std::move(callback);
    }

    /**
     * @brief Đăng ký callback nhận StepEvent
     */
    void setStepCallback(StepEventCallback callback) noexcept {
        step_callback_ = std::move(callback);
    }

    /**
     * @brief Phát lại toàn bộ dữ liệu ở tốc độ tối đa (Fastest Replay)
     */
    void playAll() noexcept;

    /**
     * @brief Tiến lên N khung hình cảm biến
     */
    bool step(size_t frame_count = 1) noexcept;

    /**
     * @brief Reset con trỏ phát lại về đầu
     */
    void reset() noexcept;

    [[nodiscard]] bool isFinished() const noexcept {
        return current_frame_idx_ >= dataset_.sensor_frames.size();
    }

    [[nodiscard]] size_t getCurrentFrameIndex() const noexcept {
        return current_frame_idx_;
    }

    [[nodiscard]] size_t getTotalFrames() const noexcept {
        return dataset_.sensor_frames.size();
    }

    [[nodiscard]] size_t getTotalSteps() const noexcept {
        return dataset_.step_events.size();
    }

    [[nodiscard]] uint64_t getCurrentTimestampNs() const noexcept {
        if (current_frame_idx_ < dataset_.sensor_frames.size()) {
            return dataset_.sensor_frames[current_frame_idx_].timestamp_ns;
        }
        return 0;
    }

    [[nodiscard]] const SyntheticDataset& getDataset() const noexcept {
        return dataset_;
    }

private:
    SyntheticDataset dataset_{};
    size_t current_frame_idx_{0};
    size_t current_step_idx_{0};
    SensorFrameCallback sensor_callback_{};
    StepEventCallback step_callback_{};
};

} // namespace survival::harness
