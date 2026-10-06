#include "survival_core/comms/packet_serializer.hpp"
#include "survival_core/math/crc32.hpp"
#include <algorithm>
#include <cmath>

namespace survival::comms {

namespace {

constexpr int32_t kMaxInt24 = 8388607; // 2^23 - 1

// Helper: Ghi int24 (3 bytes little endian)
inline void writeInt24(std::array<uint8_t, 20>& buf, size_t offset, int32_t val) noexcept {
    const uint32_t uval = static_cast<uint32_t>(val) & 0x00FFFFFFU;
    buf[offset + 0] = static_cast<uint8_t>(uval & 0xFFU);
    buf[offset + 1] = static_cast<uint8_t>((uval >> 8) & 0xFFU);
    buf[offset + 2] = static_cast<uint8_t>((uval >> 16) & 0xFFU);
}

// Helper: Đọc int24 kèm sign-extension
inline int32_t readInt24(std::span<const uint8_t, 20> buf, size_t offset) noexcept {
    uint32_t uval = static_cast<uint32_t>(buf[offset + 0]) |
                    (static_cast<uint32_t>(buf[offset + 1]) << 8) |
                    (static_cast<uint32_t>(buf[offset + 2]) << 16);
    // Nếu bit 23 là 1, sign-extend lên 32-bit signed
    if (uval & 0x00800000U) {
        uval |= 0xFF000000U;
    }
    return static_cast<int32_t>(uval);
}

} // namespace

std::array<uint8_t, Packet160Serializer::kPacketSizeBytes> Packet160Serializer::encode(
    uint32_t unix_timestamp,
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    double ce50_major_m,
    double ce50_minor_m,
    double ce_heading_deg,
    uint8_t status_flags,
    uint8_t battery_percent,
    uint8_t cluster_id) noexcept
{
    std::array<uint8_t, kPacketSizeBytes> buf{};

    // 1. Timestamp (Bytes 0..3)
    buf[0] = static_cast<uint8_t>(unix_timestamp & 0xFFU);
    buf[1] = static_cast<uint8_t>((unix_timestamp >> 8) & 0xFFU);
    buf[2] = static_cast<uint8_t>((unix_timestamp >> 16) & 0xFFU);
    buf[3] = static_cast<uint8_t>((unix_timestamp >> 24) & 0xFFU);

    // 2. Latitude [-90, 90] -> 24-bit fixed point (Bytes 4..6)
    const double clamped_lat = std::clamp(latitude_deg, -90.0, 90.0);
    const int32_t lat_fixed = static_cast<int32_t>(std::round(clamped_lat * (static_cast<double>(kMaxInt24) / 90.0)));
    writeInt24(buf, 4, lat_fixed);

    // 3. Longitude [-180, 180] -> 24-bit fixed point (Bytes 7..9)
    const double clamped_lon = std::clamp(longitude_deg, -180.0, 180.0);
    const int32_t lon_fixed = static_cast<int32_t>(std::round(clamped_lon * (static_cast<double>(kMaxInt24) / 180.0)));
    writeInt24(buf, 7, lon_fixed);

    // 4. Altitude 12-bit [0, 4095m] + Minor Ratio 4-bit [0, 15] (Bytes 10..11)
    const uint16_t alt_quant = static_cast<uint16_t>(std::clamp(static_cast<int>(std::round(altitude_m)), 0, 4095));
    double ratio = 1.0;
    if (ce50_major_m > 1e-4) {
        ratio = std::clamp(ce50_minor_m / ce50_major_m, 0.0, 1.0);
    }
    const uint8_t ratio_quant = static_cast<uint8_t>(std::round(ratio * 15.0));

    buf[10] = static_cast<uint8_t>(alt_quant & 0xFFU);
    buf[11] = static_cast<uint8_t>(((alt_quant >> 8) & 0x0FU) | ((ratio_quant & 0x0FU) << 4));

    // 5. CE50 Major Axis 8-bit [0, 255m] (Byte 12)
    const uint8_t major_quant = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(ce50_major_m)), 0, 255));
    buf[12] = major_quant;

    // 6. Heading 8-bit [0, 360) -> [0, 255] (Byte 13)
    double norm_heading = std::fmod(ce_heading_deg, 360.0);
    if (norm_heading < 0.0) norm_heading += 360.0;
    const uint8_t heading_quant = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(norm_heading * (255.0 / 360.0))), 0, 255));
    buf[13] = heading_quant;

    // 7. Status Flags (Byte 14)
    buf[14] = status_flags;

    // 8. Battery (5 bits: 0..31) + Cluster ID (3 bits: 0..7) (Byte 15)
    const uint8_t bat_5bit = static_cast<uint8_t>(std::clamp(static_cast<int>(std::round(battery_percent * (31.0 / 100.0))), 0, 31));
    const uint8_t cid_3bit = cluster_id & 0x07U;
    buf[15] = static_cast<uint8_t>((bat_5bit << 3) | cid_3bit);

    // 9. CRC32 Checksum over first 16 bytes (Bytes 16..19)
    const uint32_t crc = math::Crc32::calculate(std::span<const uint8_t>(buf.data(), 16));
    buf[16] = static_cast<uint8_t>(crc & 0xFFU);
    buf[17] = static_cast<uint8_t>((crc >> 8) & 0xFFU);
    buf[18] = static_cast<uint8_t>((crc >> 16) & 0xFFU);
    buf[19] = static_cast<uint8_t>((crc >> 24) & 0xFFU);

    return buf;
}

bool Packet160Serializer::decode(
    std::span<const uint8_t, kPacketSizeBytes> buffer,
    DecodedRescuePacket& out_packet) noexcept
{
    // 1. Kiểm tra CRC32
    const uint32_t expected_crc = static_cast<uint32_t>(buffer[16]) |
                                  (static_cast<uint32_t>(buffer[17]) << 8) |
                                  (static_cast<uint32_t>(buffer[18]) << 16) |
                                  (static_cast<uint32_t>(buffer[19]) << 24);
    const uint32_t computed_crc = math::Crc32::calculate(std::span<const uint8_t>(buffer.data(), 16));

    out_packet.crc32 = expected_crc;
    out_packet.is_crc_valid = (expected_crc == computed_crc);

    // 2. Decode Timestamp
    out_packet.unix_timestamp = static_cast<uint32_t>(buffer[0]) |
                                (static_cast<uint32_t>(buffer[1]) << 8) |
                                (static_cast<uint32_t>(buffer[2]) << 16) |
                                (static_cast<uint32_t>(buffer[3]) << 24);

    // 3. Decode Latitude
    const int32_t lat_fixed = readInt24(buffer, 4);
    out_packet.latitude_deg = static_cast<double>(lat_fixed) * (90.0 / static_cast<double>(kMaxInt24));

    // 4. Decode Longitude
    const int32_t lon_fixed = readInt24(buffer, 7);
    out_packet.longitude_deg = static_cast<double>(lon_fixed) * (180.0 / static_cast<double>(kMaxInt24));

    // 5. Decode Altitude & Minor Ratio
    const uint16_t alt_low = buffer[10];
    const uint16_t alt_high = buffer[11] & 0x0FU;
    out_packet.altitude_m = static_cast<double>((alt_high << 8) | alt_low);

    const uint8_t ratio_quant = (buffer[11] >> 4) & 0x0FU;
    out_packet.ce50_minor_ratio = static_cast<double>(ratio_quant) / 15.0;

    // 6. Decode CE50 Major & Heading
    out_packet.ce50_major_m = static_cast<double>(buffer[12]);
    out_packet.ce_heading_deg = static_cast<double>(buffer[13]) * (360.0 / 255.0);

    // 7. Decode Status & Battery / Cluster
    out_packet.status_flags = buffer[14];
    const uint8_t bat_5bit = (buffer[15] >> 3) & 0x1FU;
    out_packet.battery_percent = static_cast<uint8_t>(std::round(static_cast<double>(bat_5bit) * (100.0 / 31.0)));
    out_packet.cluster_id = buffer[15] & 0x07U;

    return out_packet.is_crc_valid;
}

} // namespace survival::comms
