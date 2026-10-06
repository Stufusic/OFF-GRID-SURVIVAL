#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <span>
#include "survival_core/data_contracts.hpp"
#include "survival_core/comms/packet_serializer.hpp"

namespace survival::comms {

struct BleAdvPacket {
    std::array<uint8_t, 31> raw_adv_data{}; // Khung dữ liệu BLE Advertising chuẩn tối đa 31 bytes
    size_t length{0};
};

class SosEgressManager {
public:
    SosEgressManager() noexcept = default;

    /**
     * @brief Đóng gói bản tin BLE Manufacturer Specific Data (Type 0xFF) từ gói tin cứu hộ 160-bit
     */
    [[nodiscard]] static BleAdvPacket buildBleAdvPayload(const OutboundRescuePacket160& packet, uint16_t company_id = 0xFFFF) noexcept;

    /**
     * @brief Sinh chuỗi tin nhắn khẩn cấp SMS ngắn gọn (Compact Base64 / ASCII URL-safe)
     */
    [[nodiscard]] static std::string buildCompactSmsPayload(const OutboundRescuePacket160& packet, const NavigationUncertaintyState& state, double battery_soc) noexcept;
};

} // namespace survival::comms
