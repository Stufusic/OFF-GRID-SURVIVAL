#pragma once

#include <cstdint>
#include <array>
#include <cstddef>
#include <algorithm>

namespace survival::math {

struct SpatialCellData {
    int64_t gx{0};
    int64_t gy{0};
    double weight_sum{0.0};
    double sum_e{0.0};
    double sum_n{0.0};
    double sum_alt{0.0};
    size_t count{0};
    bool occupied{false};
};

/**
 * @brief Bảng băm không gian tĩnh Zero-Allocation dùng Open Addressing & Linear Probing
 * Đảm bảo 0 bytes malloc trong vòng lặp hạt và thời gian truy cập O(1) worst-case có giới hạn
 */
template <size_t Capacity = 512>
class StaticSpatialHash {
public:
    StaticSpatialHash() noexcept {
        clear();
    }

    void clear() noexcept {
        for (auto& slot : table_) {
            slot.occupied = false;
            slot.weight_sum = 0.0;
            slot.sum_e = 0.0;
            slot.sum_n = 0.0;
            slot.sum_alt = 0.0;
            slot.count = 0;
        }
        size_ = 0;
    }

    void insertOrAdd(int64_t gx, int64_t gy, double weight, double easting, double northing, double alt) noexcept {
        size_t idx = hashCoords(gx, gy) % Capacity;
        const size_t start_idx = idx;

        while (table_[idx].occupied) {
            if (table_[idx].gx == gx && table_[idx].gy == gy) {
                // Đã tồn tại ô lưới -> tích lũy
                table_[idx].weight_sum += weight;
                table_[idx].sum_e += easting * weight;
                table_[idx].sum_n += northing * weight;
                table_[idx].sum_alt += alt * weight;
                table_[idx].count++;
                return;
            }
            idx = (idx + 1) % Capacity;
            if (idx == start_idx) {
                return; // Bảng đầy, dừng an toàn
            }
        }

        // Tạo ô mới
        table_[idx].gx = gx;
        table_[idx].gy = gy;
        table_[idx].weight_sum = weight;
        table_[idx].sum_e = easting * weight;
        table_[idx].sum_n = northing * weight;
        table_[idx].sum_alt = alt * weight;
        table_[idx].count = 1;
        table_[idx].occupied = true;
        size_++;
    }

    [[nodiscard]] size_t size() const noexcept {
        return size_;
    }

    [[nodiscard]] const std::array<SpatialCellData, Capacity>& getTable() const noexcept {
        return table_;
    }

private:
    std::array<SpatialCellData, Capacity> table_{};
    size_t size_{0};

    [[nodiscard]] static constexpr size_t hashCoords(int64_t x, int64_t y) noexcept {
        uint64_t h = static_cast<uint64_t>(x) * 73856093ULL ^ static_cast<uint64_t>(y) * 19349663ULL;
        h ^= h >> 33;
        h *= 0xff51afd7ed558ccdULL;
        h ^= h >> 33;
        return static_cast<size_t>(h);
    }
};

} // namespace survival::math
