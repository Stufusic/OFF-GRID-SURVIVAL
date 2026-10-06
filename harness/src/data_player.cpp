#include "survival_harness/data_player.hpp"

namespace survival::harness {

void DataPlayer::loadDataset(const SyntheticDataset& dataset) noexcept {
    dataset_ = dataset;
    reset();
}

void DataPlayer::reset() noexcept {
    current_frame_idx_ = 0;
    current_step_idx_ = 0;
}

bool DataPlayer::step(size_t frame_count) noexcept {
    if (isFinished()) {
        return false;
    }

    const size_t target_idx = std::min(current_frame_idx_ + frame_count, dataset_.sensor_frames.size());

    while (current_frame_idx_ < target_idx) {
        const auto& frame = dataset_.sensor_frames[current_frame_idx_];

        // 1. Kích hoạt các sự kiện bước chân có timestamp <= timestamp của frame hiện tại
        while (current_step_idx_ < dataset_.step_events.size() &&
               dataset_.step_events[current_step_idx_].timestamp_ns <= frame.timestamp_ns)
        {
            if (step_callback_) {
                step_callback_(dataset_.step_events[current_step_idx_]);
            }
            current_step_idx_++;
        }

        // 2. Kích hoạt callback SensorFrame
        if (sensor_callback_) {
            sensor_callback_(frame);
        }

        current_frame_idx_++;
    }

    return true;
}

void DataPlayer::playAll() noexcept {
    step(dataset_.sensor_frames.size() - current_frame_idx_);
}

} // namespace survival::harness
