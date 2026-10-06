#include "survival_core/storage/local_blackbox.hpp"
#include "survival_core/math/crc32.hpp"
#include <algorithm>
#include <fstream>

namespace survival::storage {

void LocalBlackbox::registerPlatformFileSystem(std::shared_ptr<platform::IPlatformFileSystem> fs) noexcept {
    s_platform_fs_ = std::move(fs);
}

LocalBlackbox::LocalBlackbox(std::string storage_filepath,
                             std::shared_ptr<platform::IPlatformFileSystem> platform_fs) noexcept
    : filepath_(std::move(storage_filepath))
    , platform_fs_(std::move(platform_fs))
{
}

void LocalBlackbox::pushRecord(const BlackboxRecord& record) noexcept {
    ring_buffer_[head_] = record;
    head_ = (head_ + 1) % kCapacity;
    if (count_ < kCapacity) {
        count_++;
    }
    // [H6] Kẹp tối đa kCapacity: khi ring wrap, record cũ bị ghi đè thì số record
    // "chưa flush" thực tế không thể vượt capacity (trước fix tăng vô hạn khi đĩa hỏng)
    if (unflushed_count_ < kCapacity) {
        unflushed_count_++;
    }

    // [H6] Auto-flush theo ngưỡng tích lũy + backoff luỹ thừa khi đĩa lỗi liên tiếp:
    // 100 -> 300 -> 700 -> 1500 -> 3100 ... (tránh dập open()/flush trên flash chết mỗi sample)
    if (unflushed_count_ >= auto_flush_threshold_) {
        flushToDisk();
        if (consecutive_flush_failures_ > 0) {
            const size_t shift = std::min(consecutive_flush_failures_, static_cast<size_t>(4));
            auto_flush_threshold_ = unflushed_count_ + (kAutoFlushStepInterval << shift);
        }
        // flushToDisk() thành công đã reset unflushed_count_ = 0 và auto_flush_threshold_
    }
}

bool LocalBlackbox::flushToDisk() noexcept {
    if (count_ == 0 || filepath_.empty()) {
        return false;
    }
    const size_t safe_unflushed = std::min(unflushed_count_, kCapacity);
    if (safe_unflushed == 0) {
        return true;
    }

    // [H6] in|out để ghi tiếp và xác định file mới; lưu ý chế độ in|out KHÔNG tạo
    // file khi chưa tồn tại — với file mới phải mở bằng app (append-from-zero)
    {
        std::ifstream probe(filepath_, std::ios::binary);
        if (!probe.is_open()) {
            file_header_written_ = false;
        }
    }
    std::ofstream out(filepath_, std::ios::binary | std::ios::in | std::ios::out);
    if (!out.is_open()) {
        out.open(filepath_, std::ios::binary | std::ios::app);
        if (!out.is_open()) {
            consecutive_flush_failures_++;
            return false;
        }
    }

    out.seekp(0, std::ios::end);
    if (out.tellp() <= 0 && !file_header_written_) {
        const BlackboxFileHeader hdr{};
        out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
        if (!out.good()) {
            consecutive_flush_failures_++;
            return false;
        }
        file_header_written_ = true;
    }

    const size_t start_idx = (head_ + kCapacity - safe_unflushed) % kCapacity;
    for (size_t i = 0; i < safe_unflushed; ++i) {
        const size_t idx = (start_idx + i) % kCapacity;
        const BlackboxWireRecord wire{
            ring_buffer_[idx],
            math::Crc32::calculate(std::span<const uint8_t>(
                reinterpret_cast<const uint8_t*>(&ring_buffer_[idx]), sizeof(BlackboxRecord)))
        };
        out.write(reinterpret_cast<const char*>(&wire), sizeof(wire));
        if (!out.good()) {
            consecutive_flush_failures_++;
            return false;   // [H6] KHÔNG reset unflushed_count_ — record vẫn phải flush lại
        }
    }
    out.flush();
    if (!out.good()) {
        consecutive_flush_failures_++;
        return false;
    }

    // Đồng bộ phần cứng qua Platform Abstraction Layer (Invariant 3) — kiểm tra kết quả [H6]
    const auto& fs = platform_fs_ != nullptr ? platform_fs_ : s_platform_fs_;
    if (fs != nullptr) {
        if (fs->hardwareSync(filepath_) != platform::PlatformIoResult::kSuccess) {
            consecutive_flush_failures_++;
            return false;
        }
    }

    unflushed_count_ = 0;
    consecutive_flush_failures_ = 0;
    auto_flush_threshold_ = kAutoFlushStepInterval;
    return true;
}

std::optional<std::vector<BlackboxRecord>>
LocalBlackbox::recoverRecords(const std::string& filepath) noexcept {
    std::ifstream in(filepath, std::ios::binary);
    if (!in.is_open()) {
        return std::nullopt;
    }

    BlackboxFileHeader hdr{};
    in.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!in.good() || hdr.magic != kFileMagic || hdr.version != kFormatVersion
        || hdr.record_size != sizeof(BlackboxWireRecord)) {
        return std::nullopt;
    }

    std::vector<BlackboxRecord> recovered;
    BlackboxWireRecord wire{};
    while (in.read(reinterpret_cast<char*>(&wire), sizeof(wire))) {
        const auto rec_crc = math::Crc32::calculate(std::span<const uint8_t>(
            reinterpret_cast<const uint8_t*>(&wire.record), sizeof(BlackboxRecord)));
        if (rec_crc == wire.crc32) {
            recovered.push_back(wire.record);
        }
        // Record torn-write (CRC sai): bỏ qua, tiếp tục stride — không lệch khung
    }
    if (recovered.empty()) {
        return std::nullopt;
    }
    return recovered;
}

} // namespace survival::storage
