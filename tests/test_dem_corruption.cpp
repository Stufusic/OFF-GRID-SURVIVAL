#include <gtest/gtest.h>
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/math/crc32.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

using namespace survival;
using namespace survival::environment;
using survival::math::Crc32;

namespace {
// Ghi 1 file .sdem tùy ý từ header + payload thô (không qua DemMap::saveToFile)
bool writeRawSdem(const std::string& path, const SdemFileHeader& hdr, const void* payload, size_t payload_bytes) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (payload_bytes > 0) {
        out.write(reinterpret_cast<const char*>(payload), static_cast<std::streamsize>(payload_bytes));
    }
    return out.good();
}

std::vector<char> readFileBytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<char>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
} // namespace

// [H4] Header dims rác (0xFFFFFFFF) KHÔNG được crash (bad_alloc trong noexcept = terminate)
TEST(DemCorruption, GarbageDimensionsDoesNotCrash) {
    SdemFileHeader hdr{};  // magic/version mặc định hợp lệ
    hdr.width_cells = 0xFFFFFFFFU;
    hdr.height_cells = 0xFFFFFFFFU;
    hdr.cell_size_m = 10.0F;
    hdr.payload_crc32 = 0U;

    DemMap dem;
    std::istringstream is(std::string(reinterpret_cast<const char*>(&hdr), sizeof(hdr)));
    EXPECT_FALSE(dem.loadFromStream(is));
    EXPECT_FALSE(dem.isLoaded());
}

// [H4] File cụt (payload khai báo vượt dung lượng thực tế) bị từ chối
TEST(DemCorruption, TruncatedFileRejected) {
    SdemFileHeader hdr{};
    hdr.width_cells = 100;
    hdr.height_cells = 100;  // khai báo 100x100 = 40KB payload
    hdr.cell_size_m = 10.0F;
    hdr.payload_crc32 = 0U;

    const float tiny[4] = {1.0F, 2.0F, 3.0F, 4.0F};  // chỉ 16 byte thực tế
    const std::string path = "test_dem_truncated.sdem";
    ASSERT_TRUE(writeRawSdem(path, hdr, tiny, sizeof(tiny)));

    DemMap dem;
    std::ifstream in(path, std::ios::binary);
    ASSERT_TRUE(in.is_open());
    EXPECT_FALSE(dem.loadFromStream(in));
    EXPECT_FALSE(dem.isLoaded());
    std::remove(path.c_str());
}

// [H4] w/h = 0 với CRC rỗng hợp lệ phải bị từ chối (chặn UB std::clamp hi = −1)
TEST(DemCorruption, ZeroDimensionsWithValidCrcRejected) {
    SdemFileHeader hdr{};
    hdr.width_cells = 0;
    hdr.height_cells = 0;
    hdr.cell_size_m = 10.0F;
    hdr.payload_crc32 = 0U;  // CRC32 của payload rỗng = 0

    const std::string path = "test_dem_zerodim.sdem";
    ASSERT_TRUE(writeRawSdem(path, hdr, nullptr, 0));

    DemMap dem;
    std::ifstream in(path, std::ios::binary);
    ASSERT_TRUE(in.is_open());
    EXPECT_FALSE(dem.loadFromStream(in));
    EXPECT_FALSE(dem.isLoaded());
    std::remove(path.c_str());
}

// [H4] cell_size NaN phải bị từ chối (NaN <= 0.0 là false -> lọt qua check cũ)
TEST(DemCorruption, NanCellSizeRejected) {
    SdemFileHeader hdr{};
    hdr.width_cells = 4;
    hdr.height_cells = 4;
    hdr.cell_size_m = std::numeric_limits<float>::quiet_NaN();
    hdr.payload_crc32 = 0U;

    const std::vector<float> payload(16, 100.0F);
    DemMap dem;
    std::istringstream is(std::string(reinterpret_cast<const char*>(&hdr), sizeof(hdr))
                              .append(reinterpret_cast<const char*>(payload.data()), payload.size() * 4));
    EXPECT_FALSE(dem.loadFromStream(is));
}

// [H4] File hợp lệ vẫn phải load bình thường sau khi siết validate (không hồi quy)
TEST(DemCorruption, ValidFileStillLoads) {
    DemMap src;
    ASSERT_TRUE(src.createSyntheticRamp(100.0, 100.0, 10.0, 1000.0, 1100.0));
    const std::string path = "test_dem_valid.sdem";
    ASSERT_TRUE(src.saveToFile(path));

    DemMap dem;
    EXPECT_TRUE(dem.loadFromFile(path, false));
    EXPECT_TRUE(dem.isLoaded());
    const auto z = dem.getElevationBilinear(50.0, 50.0);
    ASSERT_TRUE(z.has_value());
    EXPECT_FALSE(std::isnan(*z));
    std::remove(path.c_str());
}

// [H4] Cell NaN trong file CRC-hợp lệ phải bị coi là nodata (không trả NaN ra ngoài)
TEST(DemCorruption, NanCellTreatedAsNodata) {
    DemMap src;
    ASSERT_TRUE(src.createSyntheticRamp(40.0, 40.0, 10.0, 100.0, 200.0));
    const std::string path = "test_dem_nan.sdem";
    ASSERT_TRUE(src.saveToFile(path));

    // Patch trực tiếp byte: ô (col=1,row=1) -> NaN, tính lại CRC payload trong header
    auto bytes = readFileBytes(path);
    ASSERT_GE(bytes.size(), sizeof(SdemFileHeader));
    auto* hdr = reinterpret_cast<SdemFileHeader*>(bytes.data());
    const size_t payload_offset = sizeof(SdemFileHeader);
    const size_t nan_idx = payload_offset + (1 * hdr->width_cells + 1) * sizeof(float);
    ASSERT_LT(nan_idx + 4, bytes.size());
    constexpr uint32_t kNanBits = 0x7FC00000U;  // float NaN canonical
    std::memcpy(bytes.data() + nan_idx, &kNanBits, 4);

    hdr->payload_crc32 = Crc32::calculate(std::span<const uint8_t>(
        reinterpret_cast<const uint8_t*>(bytes.data() + payload_offset), bytes.size() - payload_offset));

    {
        std::ofstream out(path, std::ios::binary);
        ASSERT_TRUE(out.is_open());
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

    DemMap dem2;
    ASSERT_TRUE(dem2.loadFromFile(path, false));
    // Ô có NaN và vùng nội suy lan tới phải trả nullopt, tuyệt đối không trả NaN
    const auto z_bad = dem2.getElevationBilinear(10.0, 10.0);
    EXPECT_FALSE(z_bad.has_value());
    const auto z_bad_near = dem2.getElevationBilinear(0.5, 0.5);  // kề ô NaN (1,1) -> cũng bị chặn
    EXPECT_FALSE(z_bad_near.has_value());
    // Góc xa ô NaN (chỉ dùng ô 3,3) vẫn phải đọc được bình thường
    const auto z_far = dem2.getElevationBilinear(35.0, 35.0);
    ASSERT_TRUE(z_far.has_value());
    EXPECT_FALSE(std::isnan(*z_far));
    std::remove(path.c_str());
}
