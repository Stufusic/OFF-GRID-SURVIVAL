#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <array>

namespace survival::math {

namespace detail {

constexpr uint32_t kCrc32Polynomial = 0xEDB88320U;

constexpr std::array<uint32_t, 256> generateCrc32Table() noexcept {
    std::array<uint32_t, 256> table{};
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
            if (c & 1U) {
                c = kCrc32Polynomial ^ (c >> 1);
            } else {
                c >>= 1;
            }
        }
        table[i] = c;
    }
    return table;
}

inline constexpr std::array<uint32_t, 256> kCrc32Table = generateCrc32Table();

} // namespace detail

/**
 * @brief Bảng tra cứu và thuật toán tính CRC32 chuẩn IEEE 802.3 (Polynomial 0xEDB88320)
 * Dùng cho kiểm tra toàn vẹn gói tin cứu hộ 160-bit (OutboundRescuePacket160)
 */
class Crc32 {
public:
    static constexpr uint32_t kPolynomial = detail::kCrc32Polynomial;

    [[nodiscard]] static constexpr uint32_t calculate(std::span<const uint8_t> data) noexcept {
        uint32_t crc = 0xFFFFFFFFU;
        for (const uint8_t byte : data) {
            crc = (crc >> 8) ^ detail::kCrc32Table[(crc ^ byte) & 0xFFU];
        }
        return crc ^ 0xFFFFFFFFU;
    }

    [[nodiscard]] static constexpr uint32_t calculate(const void* data, size_t length) noexcept {
        return calculate(std::span<const uint8_t>(static_cast<const uint8_t*>(data), length));
    }
};

} // namespace survival::math
