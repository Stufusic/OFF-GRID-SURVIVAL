#include <gtest/gtest.h>
#include "survival_core/storage/local_blackbox.hpp"
#include "survival_core/math/crc32.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

using namespace survival;
using namespace survival::storage;
using survival::math::Crc32;

namespace {
BlackboxRecord makeRecord(uint64_t ts_ns) {
    BlackboxRecord rec{};
    rec.timestamp_ns = ts_ns;
    rec.easting_m = static_cast<float>(ts_ns % 100000);
    rec.northing_m = static_cast<float>(ts_ns % 99991);
    rec.altitude_m = 1500.0F;
    rec.ce95_m = 42.0F;
    rec.heading_deg = 123.5F;
    rec.battery_soc = 88.0F;
    rec.safety_state = 1;
    rec.flags = 2;
    return rec;
}

// PAL giả lập: đếm số lần gọi hardwareSync, trả kết quả cấu hình được
class SpyPal : public survival::platform::IPlatformFileSystem {
public:
    int sync_calls{0};
    survival::platform::PlatformIoResult result{survival::platform::PlatformIoResult::kSuccess};

    survival::platform::PlatformIoResult hardwareSync(const std::string&) noexcept override {
        sync_calls++;
        return result;
    }
};

std::vector<char> readFileBytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<char>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
} // namespace

// [H6] Đĩa hỏng: flush phải trả false, giữ nguyên unflushed_count_, không reset mất dữ liệu
TEST(BlackboxFlush, FailurePreservesUnflushedCount) {
    // Path trong thư mục không tồn tại -> ofstream mở thất bại chắc chắn
    LocalBlackbox box("no_such_dir_4f3a/blackbox.bin");
    for (int i = 0; i < 150; ++i) {
        box.pushRecord(makeRecord(static_cast<uint64_t>(i) * 1000000ULL));
    }

    EXPECT_FALSE(box.flushToDisk());
    EXPECT_GT(box.getUnflushedCount(), 0U);
    EXPECT_LE(box.getUnflushedCount(), LocalBlackbox::kCapacity);  // không tăng vô hạn
    EXPECT_GT(box.getConsecutiveFlushFailures(), 0U);
}

// [H6] Flush thành công: reset counter, file có header v2, recoverRecords đọc đúng dữ liệu
TEST(BlackboxFlush, SuccessResetsCounterAndRoundtrips) {
    const std::string path = "test_blackbox_ok.bin";
    LocalBlackbox box(path);

    for (int i = 0; i < 120; ++i) {
        box.pushRecord(makeRecord(static_cast<uint64_t>(i) * 1000000ULL));
    }
    // pushRecord tự flush tại mốc 100 -> còn 20 unflushed
    ASSERT_TRUE(box.flushToDisk());
    EXPECT_EQ(box.getUnflushedCount(), 0U);

    // File: 8 byte header + 120 * 40 byte wire record
    const auto bytes = readFileBytes(path);
    ASSERT_GE(bytes.size(), sizeof(BlackboxFileHeader));
    const auto* hdr = reinterpret_cast<const BlackboxFileHeader*>(bytes.data());
    EXPECT_EQ(hdr->magic, LocalBlackbox::kFileMagic);
    EXPECT_EQ(hdr->version, LocalBlackbox::kFormatVersion);
    EXPECT_EQ(hdr->record_size, sizeof(BlackboxWireRecord));
    EXPECT_EQ(bytes.size(), sizeof(BlackboxFileHeader) + 120U * sizeof(BlackboxWireRecord));

    // Phục hồi toàn bộ và so khớp nội dung
    const auto recovered = LocalBlackbox::recoverRecords(path);
    ASSERT_TRUE(recovered.has_value());
    ASSERT_EQ(recovered->size(), 120U);
    for (int i = 0; i < 120; ++i) {
        EXPECT_EQ((*recovered)[static_cast<size_t>(i)].timestamp_ns,
                  static_cast<uint64_t>(i) * 1000000ULL);
        EXPECT_NEAR((*recovered)[static_cast<size_t>(i)].heading_deg, 123.5F, 1e-6);
    }
    std::remove(path.c_str());
}

// [H6] Backoff luỹ thừa: 3000 push trên đĩa sync-fail chỉ được phép thử sync ~4 lần
TEST(BlackboxFlush, BackoffReducesRetryRate) {
    const std::string path = "test_blackbox_backoff.bin";
    auto pal = std::make_shared<SpyPal>();
    pal->result = survival::platform::PlatformIoResult::kSyncFailed;

    LocalBlackbox box(path, pal);
    for (int i = 0; i < 3000; ++i) {
        box.pushRecord(makeRecord(static_cast<uint64_t>(i) * 1000000ULL));
    }
    // Auto-flush tại unflushed = 100, 300, 700, 1500 (kế tiếp 3100 > 3000)
    EXPECT_EQ(pal->sync_calls, 4);
    std::remove(path.c_str());
}

// [H6] Torn-write: hỏng 1 record giữa file chỉ mất đúng record đó, phần sau phục hồi được
TEST(BlackboxRecovery, TornWriteLosesOnlyOneRecord) {
    const std::string path = "test_blackbox_torn.bin";
    LocalBlackbox box(path);
    for (int i = 0; i < 100; ++i) {
        box.pushRecord(makeRecord(static_cast<uint64_t>(i) * 1000000ULL));
    }
    ASSERT_TRUE(box.flushToDisk());

    // Hủy hoại payload của record #50 (đổi byte timestamp) mà KHÔNG sửa CRC
    auto bytes = readFileBytes(path);
    const size_t rec50_offset = sizeof(BlackboxFileHeader) + 50U * sizeof(BlackboxWireRecord);
    ASSERT_LT(rec50_offset, bytes.size());
    bytes[rec50_offset] ^= 0xFF;
    {
        std::ofstream out(path, std::ios::binary);
        ASSERT_TRUE(out.is_open());
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

    const auto recovered = LocalBlackbox::recoverRecords(path);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(recovered->size(), 99U);
    // Record 49 và 51 vẫn nguyên vẹn, thứ tự bảo toàn
    EXPECT_EQ((*recovered)[49].timestamp_ns, 49ULL * 1000000ULL);
    EXPECT_EQ((*recovered)[50].timestamp_ns, 51ULL * 1000000ULL);
    std::remove(path.c_str());
}

// [H6] File rác / rỗng phải bị từ chối rõ ràng
TEST(BlackboxRecovery, RejectsBadMagic) {
    const std::string path = "test_blackbox_garbage.bin";
    {
        std::ofstream out(path, std::ios::binary);
        const char garbage[64] = {};
        out.write(garbage, sizeof(garbage));
    }
    EXPECT_FALSE(LocalBlackbox::recoverRecords(path).has_value());
    std::remove(path.c_str());

    // File rỗng
    const std::string empty_path = "test_blackbox_empty.bin";
    {
        std::ofstream out(empty_path, std::ios::binary);
    }
    EXPECT_FALSE(LocalBlackbox::recoverRecords(empty_path).has_value());
    std::remove(empty_path.c_str());
}

// [H6] Ring buffer wrap + append-only log: file chứa ĐỦ 6000 record, mỗi record
// đúng 1 lần (Bug 10 cũ: underflow khiến ghi lặp lại hàng nghìn bản ghi trùng)
TEST(BlackboxRing, WraparoundFlushesEachRecordExactlyOnce) {
    const std::string path = "test_blackbox_wrap.bin";
    LocalBlackbox box(path);
    for (size_t i = 0; i < 6000; ++i) {
        box.pushRecord(makeRecord(i * 1000000ULL));
    }
    EXPECT_EQ(box.getRecordCount(), LocalBlackbox::kCapacity);   // RAM: 5000 mới nhất
    EXPECT_TRUE(box.flushToDisk());

    const auto recovered = LocalBlackbox::recoverRecords(path);
    ASSERT_TRUE(recovered.has_value());
    // File append-only: đủ 6000 record, không trùng, không ghi lặp
    EXPECT_EQ(recovered->size(), 6000U);
    EXPECT_EQ((*recovered)[0].timestamp_ns, 0ULL);
    EXPECT_EQ(recovered->back().timestamp_ns, 5999ULL * 1000000ULL);
    // Kiểm tra không có timestamp trùng (chống ghi lặp kiểu Bug 10)
    for (size_t i = 1; i < recovered->size(); ++i) {
        EXPECT_GT((*recovered)[i].timestamp_ns, (*recovered)[i - 1].timestamp_ns);
    }
    std::remove(path.c_str());
}
