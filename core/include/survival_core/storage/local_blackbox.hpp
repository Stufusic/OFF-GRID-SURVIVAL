#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <span>
#include <vector>
#include "survival_core/types.hpp"
#include "survival_core/data_contracts.hpp"
#include "survival_core/platform/platform_file_system.hpp"

namespace survival::storage {

#pragma pack(push, 1)
struct BlackboxRecord {
    uint64_t timestamp_ns{0};
    float easting_m{0.0F};
    float northing_m{0.0F};
    float altitude_m{0.0F};
    float ce95_m{0.0F};
    float heading_deg{0.0F};
    float battery_soc{100.0F};
    uint8_t safety_state{0};
    uint8_t flags{0};
    uint16_t reserved{0};
};
#pragma pack(pop)

static_assert(sizeof(BlackboxRecord) == 36, "BlackboxRecord must be exactly 36 bytes packed");

// ============================================================================
// [H6] Định dạng file Blackbox v2: header 8 byte + record 40 byte kèm CRC32.
// Torn-write khi mất điện chỉ hỏng đúng record đó — phần sau vẫn phục hồi được.
// ============================================================================
#pragma pack(push, 1)
struct BlackboxFileHeader {
    uint32_t magic{0x31584242};      // "BBX1" (little-endian)
    uint16_t version{2};
    uint16_t record_size{40};        // sizeof(BlackboxWireRecord)
};
struct BlackboxWireRecord {
    BlackboxRecord record;           // 36 bytes
    uint32_t crc32{0};               // CRC32 của 36 byte record
};
#pragma pack(pop)

static_assert(sizeof(BlackboxFileHeader) == 8, "BlackboxFileHeader must be exactly 8 bytes packed");
static_assert(sizeof(BlackboxWireRecord) == 40, "BlackboxWireRecord must be exactly 40 bytes packed");

class LocalBlackbox {
public:
    static constexpr size_t kCapacity = 5000;
    static constexpr size_t kAutoFlushStepInterval = 100;
    static constexpr uint32_t kFileMagic = 0x31584242;   // "BBX1"
    static constexpr uint16_t kFormatVersion = 2;

    explicit LocalBlackbox(std::string storage_filepath = "survival_blackbox.bin",
                           std::shared_ptr<platform::IPlatformFileSystem> platform_fs = nullptr) noexcept;

    static void registerPlatformFileSystem(std::shared_ptr<platform::IPlatformFileSystem> fs) noexcept;

    /**
     * @brief Ghi 1 mẫu dữ liệu vào Ring Buffer trên RAM.
     * [H6] unflushed_count_ kẹp tối đa kCapacity (chống tăng vô hạn khi đĩa hỏng)
     * và auto-flush có backoff luỹ thừa theo số lần flush lỗi liên tiếp.
     */
    void pushRecord(const BlackboxRecord& record) noexcept;

    /**
     * @brief Đẩy toàn bộ dữ liệu từ RAM xuống tệp Flash Storage.
     * [H6] Chỉ reset unflushed_count_ khi ghi + sync THÀNH CÔNG; khi lỗi trả về
     * false, giữ nguyên counter và tăng bộ đếm lỗi liên tiếp.
     */
    bool flushToDisk() noexcept;

    /**
     * @brief [H6] Phục hồi toàn bộ record hợp lệ từ file blackbox (hậu kiểm sau crash).
     * Record sai CRC do torn-write bị bỏ qua; các record sau vẫn đọc được.
     * @return nullopt nếu file không hợp lệ (magic/version/record_size sai) hoặc không có record nào.
     */
    [[nodiscard]] static std::optional<std::vector<BlackboxRecord>> recoverRecords(const std::string& filepath) noexcept;

    [[nodiscard]] size_t getRecordCount() const noexcept {
        return count_;
    }

    [[nodiscard]] size_t getUnflushedCount() const noexcept {
        return unflushed_count_;
    }

    [[nodiscard]] size_t getConsecutiveFlushFailures() const noexcept {
        return consecutive_flush_failures_;
    }

    void setStorageFilepath(std::string filepath) noexcept {
        filepath_ = std::move(filepath);
        file_header_written_ = false;
    }

    void clear() noexcept {
        head_ = 0;
        count_ = 0;
        unflushed_count_ = 0;
        consecutive_flush_failures_ = 0;
        auto_flush_threshold_ = kAutoFlushStepInterval;
        file_header_written_ = false;
    }

private:
    std::string filepath_{"survival_blackbox.bin"};
    std::shared_ptr<platform::IPlatformFileSystem> platform_fs_{nullptr};
    std::array<BlackboxRecord, kCapacity> ring_buffer_{};
    size_t head_{0};
    size_t count_{0};
    size_t unflushed_count_{0};
    size_t consecutive_flush_failures_{0};
    size_t auto_flush_threshold_{kAutoFlushStepInterval};  // [H6] ngưỡng auto-flush tích lũy (backoff)
    bool file_header_written_{false};

    static inline std::shared_ptr<platform::IPlatformFileSystem> s_platform_fs_{nullptr};
};

} // namespace survival::storage
