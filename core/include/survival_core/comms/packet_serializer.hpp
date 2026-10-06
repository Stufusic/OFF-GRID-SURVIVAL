#pragma once

#include <cstdint>
#include <array>
#include <span>
#include "survival_core/data_contracts.hpp"

namespace survival::comms {

/**
 * @brief Cấu trúc dữ liệu giải mã từ gói tin cứu hộ 160-bit (20 bytes)
 */
struct DecodedRescuePacket {
    uint32_t unix_timestamp{0};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    double ce50_major_m{0.0};
    double ce50_minor_ratio{1.0}; // b/a trong khoảng [0.0, 1.0]
    double ce_heading_deg{0.0};
    uint8_t status_flags{0};
    uint8_t battery_percent{100};
    uint8_t cluster_id{0};
    uint32_t crc32{0};
    bool is_crc_valid{false};
};

/**
 * @brief Bộ mã hóa/giải mã tuần tự bit tường minh (Explicit Bit-Shift Serialization)
 * Đảm bảo 100% Endianness-safety và Cross-platform compatibility (Windows, Android, Linux)
 */
class Packet160Serializer {
public:
    static constexpr size_t kPacketSizeBytes = 20;

    /**
     * @brief Mã hóa gói tin 160-bit từ dữ liệu điều hướng và tọa độ
     */
    [[nodiscard]] static std::array<uint8_t, kPacketSizeBytes> encode(
        uint32_t unix_timestamp,
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        double ce50_major_m,
        double ce50_minor_m,
        double ce_heading_deg,
        uint8_t status_flags,
        uint8_t battery_percent,
        uint8_t cluster_id) noexcept;

    /**
     * @brief Giải mã mảng 20 bytes thành cấu trúc DecodedRescuePacket
     */
    [[nodiscard]] static bool decode(
        std::span<const uint8_t, kPacketSizeBytes> buffer,
        DecodedRescuePacket& out_packet) noexcept;
};

} // namespace survival::comms
