#pragma once

#include <cmath>

namespace survival::kinematics {

/**
 * @brief Bộ lọc số IIR Butterworth bậc 2 dải thông 0.5 - 3.0 Hz, TỰ THÍCH ỨNG tần số lấy mẫu
 * [C1] Bản cũ khóa hệ số cho Fs = 100 Hz: chạy ở 50 Hz (MotionSense) dải thông
 * biến thành [0.25, 1.5] Hz — tần số bước 1.6-2.0 Hz bị suy hao méo dạng sóng,
 * phát hiện bước sai nhịp. Hệ số được tính lại (RBJ biquad) khi Fs quan sát
 * được lệch > 8% so với Fs hiện tại.
 * Có độ lợi chuẩn xác 1.0 (0 dB) tại dải tần số bước chân.
 */
class BandpassFilter {
public:
    explicit BandpassFilter(double sample_rate_hz = 100.0) noexcept {
        designCoefficients(sample_rate_hz);
        reset();
    }

    /// Cập nhật tần số lấy mẫu (chỉ thiết kế lại khi lệch đáng kể để tránh recompute mỗi mẫu)
    void setSampleRate(double sample_rate_hz) noexcept {
        if (sample_rate_hz < 10.0 || sample_rate_hz > 2000.0) {
            return; // ngoài dải hợp lệ — giữ nguyên
        }
        const double ratio = sample_rate_hz / fs_;
        if (ratio > 1.08 || ratio < 0.925) {
            designCoefficients(sample_rate_hz);
            reset();
        }
    }

    /**
     * @brief Lọc 1 mẫu tín hiệu đầu vào x[n]
     */
    [[nodiscard]] double process(double input) noexcept {
        // Direct Form II:
        // w[n] = x[n] - a1 * w[n-1] - a2 * w[n-2]
        // y[n] = b0 * w[n] + b1 * w[n-1] + b2 * w[n-2]
        const double w = input - a1_ * w1_ - a2_ * w2_;
        const double y = b0_ * w + b1_ * w1_ + b2_ * w2_;

        w2_ = w1_;
        w1_ = w;

        return y;
    }

    void reset() noexcept {
        w1_ = 0.0;
        w2_ = 0.0;
    }

    [[nodiscard]] double getSampleRate() const noexcept {
        return fs_;
    }

private:
    // Thiết kế biquad bandpass RBJ cho dải [0.5, 3.0] Hz:
    // f0 = sqrt(f_lo * f_hi), Q = f0 / (f_hi - f_lo)
    void designCoefficients(double fs) noexcept {
        fs_ = fs;
        constexpr double kFlo = 0.5;
        constexpr double kFhi = 3.0;
        const double f0 = std::sqrt(kFlo * kFhi);          // 1.2247 Hz
        const double q = f0 / (kFhi - kFlo);               // 0.4899
        const double w0 = 2.0 * 3.14159265358979323846 * f0 / fs;
        const double alpha = std::sin(w0) / (2.0 * q);
        const double a0 = 1.0 + alpha;

        b0_ = alpha / a0;
        b1_ = 0.0;
        b2_ = -alpha / a0;
        a1_ = (-2.0 * std::cos(w0)) / a0;
        a2_ = (1.0 - alpha) / a0;
    }

    double fs_{100.0};
    double b0_{0.07296};
    double b1_{0.0};
    double b2_{-0.07296};
    double a1_{-1.8486};
    double a2_{0.8541};

    double w1_{0.0};
    double w2_{0.0};
};

} // namespace survival::kinematics
