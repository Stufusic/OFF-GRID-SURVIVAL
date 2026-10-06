// ============================================================================
// HUD MATH — toán thuần của dashboard (không phụ thuộc ImGui/Win32) để unit
// test được (khuyến nghị rvdesktop.md: extract testable HUD math).
// ============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace desktop {

// ----------------------------------------------------------------------------
// MapView: biến đổi world (East/North, mét) <-> screen (px) với zoom + pan.
// Trục North hướng LÊN màn hình (Y-flip chuẩn bản đồ).
// ----------------------------------------------------------------------------
struct MapView {
    double center_east_m{1000.0};   // tâm nhìn (world)
    double center_north_m{1000.0};
    double pixels_per_m{0.30};      // zoom

    [[nodiscard]] double viewportWidthM(double vp_w_px) const noexcept {
        return (vp_w_px > 0.0) ? vp_w_px / pixels_per_m : 0.0;
    }

    [[nodiscard]] float screenX(double east_m, double vp_w_px) const noexcept {
        return static_cast<float>((east_m - center_east_m) * pixels_per_m + vp_w_px * 0.5);
    }

    [[nodiscard]] float screenY(double north_m, double vp_h_px) const noexcept {
        // Y-flip: North lên trên
        return static_cast<float>(-(north_m - center_north_m) * pixels_per_m + vp_h_px * 0.5);
    }

    [[nodiscard]] double worldEast(double screen_x_px, double vp_w_px) const noexcept {
        return (screen_x_px - vp_w_px * 0.5) / pixels_per_m + center_east_m;
    }

    [[nodiscard]] double worldNorth(double screen_y_px, double vp_h_px) const noexcept {
        return -(screen_y_px - vp_h_px * 0.5) / pixels_per_m + center_north_m;
    }

    // Kéo chuột theo hướng nào thì nội dung bản đồ đi theo hướng đó:
    // kéo phải 100px -> trung tâm nhìn dịch TRÁI 100m (thấy phần bên trái).
    void panPixels(double dx_px, double dy_px) noexcept {
        center_east_m -= dx_px / pixels_per_m;
        center_north_m += dy_px / pixels_per_m;  // screen Y đi xuống = kéo nội dung xuống
    }

    void zoomAt(double screen_x_px, double screen_y_px, double vp_w_px, double vp_h_px,
                double factor) noexcept {
        const double east_before = worldEast(screen_x_px, vp_w_px);
        const double north_before = worldNorth(screen_y_px, vp_h_px);
        pixels_per_m = std::clamp(pixels_per_m * factor, 0.02, 8.0);
        // Giữ điểm world dưới con trỏ đứng yên
        const double east_after = worldEast(screen_x_px, vp_w_px);
        const double north_after = worldNorth(screen_y_px, vp_h_px);
        center_east_m += east_before - east_after;
        center_north_m += north_before - north_after;
    }

    // Fit một tập điểm world vào viewport (padding px)
    void fitTo(const std::vector<std::pair<double, double>>& points,
               double vp_w_px, double vp_h_px, double pad_px = 40.0) noexcept {
        if (points.empty() || vp_w_px <= 2 * pad_px || vp_h_px <= 2 * pad_px) return;
        double min_e = points.front().first, max_e = min_e;
        double min_n = points.front().second, max_n = min_n;
        for (const auto& [e, n] : points) {
            min_e = std::min(min_e, e); max_e = std::max(max_e, e);
            min_n = std::min(min_n, n); max_n = std::max(max_n, n);
        }
        const double span_e = std::max(1.0, max_e - min_e);
        const double span_n = std::max(1.0, max_n - min_n);
        pixels_per_m = std::min((vp_w_px - 2 * pad_px) / span_e, (vp_h_px - 2 * pad_px) / span_n);
        pixels_per_m = std::clamp(pixels_per_m, 0.02, 8.0);
        center_east_m = 0.5 * (min_e + max_e);
        center_north_m = 0.5 * (min_n + max_n);
    }
};

// ----------------------------------------------------------------------------
// Ellipse CE95: 64 điểm biên (semi-major/minor + heading rad, North-based CW)
// ----------------------------------------------------------------------------
[[nodiscard]] inline std::array<std::pair<float, float>, 64> ellipseOutline(
    double center_east_m, double center_north_m,
    double semi_major_m, double semi_minor_m, double heading_rad) noexcept {
    std::array<std::pair<float, float>, 64> out{};
    const double cos_h = std::cos(heading_rad);
    const double sin_h = std::sin(heading_rad);
    for (size_t i = 0; i < 64; ++i) {
        const double t = static_cast<double>(i) / 64.0 * 2.0 * 3.14159265358979323846;
        const double a = semi_major_m * std::cos(t);
        const double b = semi_minor_m * std::sin(t);
        // Trục major quay theo heading (từ Bắc, chiều kim đồng hồ):
        out[i] = {static_cast<float>(center_east_m + a * sin_h + b * cos_h),
                  static_cast<float>(center_north_m + a * cos_h - b * sin_h)};
    }
    return out;
}

// ----------------------------------------------------------------------------
// RingBuffer cho ImPlot time-series
// ----------------------------------------------------------------------------
template <typename T, size_t N>
class RingBuffer {
public:
    void push(T v) noexcept {
        data_[head_] = v;
        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
    }
    [[nodiscard]] size_t size() const noexcept { return count_; }
    [[nodiscard]] constexpr size_t capacity() const noexcept { return N; }
    void clear() noexcept { head_ = 0; count_ = 0; }

    // Copy ra mảng tuần tự (cũ -> mới) cho ImPlot::PlotLine
    [[nodiscard]] size_t copyOut(T* dst, size_t max_len) const noexcept {
        const size_t n = std::min(count_, max_len);
        const size_t start = (count_ < N) ? 0 : head_;
        for (size_t i = 0; i < n; ++i) dst[i] = data_[(start + i) % N];
        return n;
    }
    [[nodiscard]] T latestOr(T fallback) const noexcept {
        return (count_ == 0) ? fallback : data_[(head_ + N - 1) % N];
    }

private:
    std::array<T, N> data_{};
    size_t head_{0};
    size_t count_{0};
};

// ----------------------------------------------------------------------------
// Geo: địa lý thẳng (equirectangular) quanh 1 gốc cấu hình được
// ----------------------------------------------------------------------------
struct GeoOrigin {
    double base_lat_deg{21.0285};   // default Hà Nội (chỉnh được trong UI)
    double base_lon_deg{105.8542};

    [[nodiscard]] std::pair<double, double> enToLatLon(double east_m, double north_m) const noexcept {
        constexpr double kMetersPerDegLat = 111320.0;
        const double m_per_deg_lon = 111320.0 * std::cos(base_lat_deg * 3.14159265358979323846 / 180.0);
        return {base_lat_deg + north_m / kMetersPerDegLat,
                base_lon_deg + east_m / std::max(1.0, m_per_deg_lon)};
    }
};

// Hex dump 20 byte dạng "XX XX ..." (chuỗi UTF-8 cho ImGui)
[[nodiscard]] inline std::string toHexDump(const unsigned char* data, size_t len) noexcept {
    static const char* kHex = "0123456789ABCDEF";
    std::string out;
    out.reserve(len * 3);
    for (size_t i = 0; i < len; ++i) {
        out += kHex[data[i] >> 4];
        out += kHex[data[i] & 0xF];
        if (i + 1 < len) out += ' ';
    }
    return out;
}

// Nhãn hoạt động (khớp MotionActivity enum trong types.hpp)
[[nodiscard]] inline const char* activityLabel(uint8_t act) noexcept {
    switch (act) {
        case 0: return "DUNG YEN";
        case 1: return "DI BO";
        case 2: return "CHAY";
        case 3: return "LEN THANG";
        case 4: return "XUONG THANG";
        case 5: return "CHIEU NGHI";
        case 6: return "THANG CUON";
        case 7: return "DAP XE";
        case 8: return "XE CO";
        case 9: return "THANG MAY";
        case 10: return "CAP Treo";
        default: return "KHONG RO";
    }
}

} // namespace desktop
