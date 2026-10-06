#include <gtest/gtest.h>
#include "survival_core/kinematics/bandpass_filter.hpp"
#include "survival_core/kinematics/step_detector.hpp"
#include "survival_core/types.hpp"
#include <cmath>

using namespace survival;
using namespace survival::kinematics;

TEST(BandpassFilterTest, Passband1_8HzTransmission) {
    BandpassFilter filter;

    // Tín hiệu nhịp bước 1.8 Hz tại Fs = 100 Hz
    constexpr double fs = 100.0;
    constexpr double freq = 1.8;
    double max_output = 0.0;

    for (int i = 0; i < 300; ++i) { // 3 giây
        const double t = static_cast<double>(i) / fs;
        const double input = 2.0 * std::sin(kTwoPi * freq * t);
        const double output = filter.process(input);
        if (i > 100) { // Chờ qua giai đoạn quá độ
            max_output = std::max(max_output, std::abs(output));
        }
    }

    // Tín hiệu 1.8 Hz phải truyền qua tốt (biên độ > 1.0)
    EXPECT_GT(max_output, 1.0);
}

TEST(BandpassFilterTest, HighFrequency20HzAttenuation) {
    BandpassFilter filter;

    // Nhiễu rung tần số cao 20.0 Hz tại Fs = 100 Hz
    constexpr double fs = 100.0;
    constexpr double freq = 20.0;
    double max_output = 0.0;

    for (int i = 0; i < 300; ++i) {
        const double t = static_cast<double>(i) / fs;
        const double input = 2.0 * std::sin(kTwoPi * freq * t);
        const double output = filter.process(input);
        if (i > 100) {
            max_output = std::max(max_output, std::abs(output));
        }
    }

    // Tần số 20 Hz phải bị suy hao mạnh (< 0.5 từ biên độ 2.0)
    EXPECT_LT(max_output, 0.5);
}

TEST(StepDetectorTest, SyntheticWalking18Steps) {
    StepDetector detector;

    // Giả lập 10 giây đi bộ với nhịp 1.8 Hz (18 bước)
    constexpr double fs = 100.0;
    constexpr double step_freq = 1.8;
    constexpr int total_samples = 1000;

    size_t detected_steps = 0;

    for (int i = 0; i < total_samples; ++i) {
        const double t = static_cast<double>(i) / fs;
        const uint64_t timestamp_ns = static_cast<uint64_t>(t * 1e9);

        // Gia tốc dao động nhịp bước: a_v = 2.5 * sin(2*pi*1.8*t)
        const double a_v = 2.5 * std::sin(kTwoPi * step_freq * t);

        if (detector.processSample(a_v, timestamp_ns)) {
            detected_steps++;
        }
    }

    // Phải nhận diện được 17 hoặc 18 bước
    EXPECT_GE(detected_steps, 17U);
    EXPECT_LE(detected_steps, 19U);
    EXPECT_NEAR(detector.getLastStepFeatures().cadence_hz, 1.8, 0.2);
}
