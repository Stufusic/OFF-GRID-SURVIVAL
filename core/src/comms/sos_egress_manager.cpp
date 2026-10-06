#include "survival_core/comms/sos_egress_manager.hpp"
#include "survival_core/comms/packet_serializer.hpp"
#include <cstdio>

namespace survival::comms {

BleAdvPacket SosEgressManager::buildBleAdvPayload(const OutboundRescuePacket160& packet, uint16_t company_id) noexcept {
    BleAdvPacket adv{};

    // Cấu trúc BLE AD Element cho Manufacturer Specific Data:
    // Byte 0: Length (1 byte Type + 2 bytes Company ID + 20 bytes Payload = 23 bytes)
    // Byte 1: AD Type (0xFF = Manufacturer Specific Data)
    // Byte 2-3: Company Identifier (Little-Endian)
    // Byte 4-23: 20 bytes Rescue Packet Payload
    adv.raw_adv_data[0] = 23; // AD Length
    adv.raw_adv_data[1] = 0xFF; // AD Type: Manufacturer Specific
    adv.raw_adv_data[2] = static_cast<uint8_t>(company_id & 0xFF);
    adv.raw_adv_data[3] = static_cast<uint8_t>((company_id >> 8) & 0xFF);

    for (size_t i = 0; i < 20; ++i) {
        adv.raw_adv_data[4 + i] = packet.data[i];
    }

    adv.length = 24; // 1 + 23 = 24 bytes
    return adv;
}

std::string SosEgressManager::buildCompactSmsPayload(const OutboundRescuePacket160& packet, const NavigationUncertaintyState& state, double battery_soc) noexcept {
    char buffer[256];
    const auto& c0 = state.clusters[0];

    // Decode tọa độ WGS-84 từ packet
    // [M9] Bit NO_FIX (0x80, byte status_flags): chưa từng có điểm neo hợp lệ ->
    // TUYỆT ĐỐI không phát tọa độ (0,0) 'Null Island' trong tin nhắn cứu hộ.
    // Decode thất bại (CRC sai) cũng coi như NO-FIX để an toàn tuyệt đối.
    DecodedRescuePacket decoded{};
    const bool decode_ok = Packet160Serializer::decode(packet.data, decoded);
    double lat = 0.0;
    double lon = 0.0;
    if (decode_ok) {
        lat = decoded.latitude_deg;
        lon = decoded.longitude_deg;
    }

    const char* mode_str = "GPS";
    switch (state.nav_mode) {
        case NavigationMode::kGpsAided: mode_str = "GPS"; break;
        case NavigationMode::kPdrAided: mode_str = "PDR"; break;
        case NavigationMode::kTrnAided: mode_str = "TRN"; break;
        case NavigationMode::kDegraded: mode_str = "DEGRADED"; break;
        case NavigationMode::kLost: mode_str = "LOST"; break;
        case NavigationMode::kEmergency: mode_str = "EMERGENCY"; break;
    }

    const bool no_fix = !decode_ok || (decoded.status_flags & 0x80U) != 0U;
    if (no_fix) {
        std::snprintf(
            buffer,
            sizeof(buffer),
            "SOS | NO-FIX | NAV:%s | BAT:%.0f%% | AGE:%.0fs | CE95:%.0fm",
            mode_str,
            battery_soc,
            state.position_age_s,
            c0.ce95_semi_major_m);
        return std::string(buffer);
    }

    // Tọa độ WGS-84 là nguồn dữ liệu chân lý duy nhất trong tin nhắn SMS cứu nạn
    std::snprintf(
        buffer,
        sizeof(buffer),
        "SOS | LAT:%.6f | LON:%.6f | ALT:%.0fm | CE95:%.0fm | NAV:%s | BAT:%.0f%% | AGE:%.0fs | https://maps.google.com/?q=%.6f,%.6f",
        lat,
        lon,
        c0.center_altitude_m,
        c0.ce95_semi_major_m,
        mode_str,
        battery_soc,
        state.position_age_s,
        lat,
        lon
    );

    return std::string(buffer);
}

} // namespace survival::comms
