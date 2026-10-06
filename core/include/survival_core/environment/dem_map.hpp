#pragma once

#include <cstdint>
#include <cmath>
#include <vector>
#include <span>
#include <string>
#include <optional>
#include <utility>
#include <istream>
#include "survival_core/types.hpp"
#include "survival_core/platform/platform_memory_map.hpp"

namespace survival::environment {

#pragma pack(push, 1)
struct SdemFileHeader {
    uint32_t magic{0x5344454D};      // "SDEM"
    uint16_t version{1};             // Phiên bản 1
    uint16_t header_size{64};        // 64 bytes
    double origin_easting_m{0.0};
    double origin_northing_m{0.0};
    double origin_lat_rad{0.0};
    double origin_lon_rad{0.0};
    double cell_size_m{10.0};
    uint32_t width_cells{0};
    uint32_t height_cells{0};
    float nodata_value{-9999.0F};
    uint32_t payload_crc32{0};
};
#pragma pack(pop)

static_assert(sizeof(SdemFileHeader) == 64, "SdemFileHeader must be exactly 64 bytes");

struct DemHeader {
    double origin_easting_m{0.0};    // Tọa độ ENU góc Tây-Nam (Tây) [m]
    double origin_northing_m{0.0};   // Tọa độ ENU góc Tây-Nam (Nam) [m]
    double origin_lat_rad{0.0};      // Vĩ độ WGS84 gốc [rad]
    double origin_lon_rad{0.0};      // Kinh độ WGS84 gốc [rad]
    double cell_size_m{10.0};        // Độ phân giải ô lưới [m/pixel]
    size_t width_cells{0};           // Số cột (trục Easting)
    size_t height_cells{0};          // Số hàng (trục Northing)
    float nodata_value{-9999.0F};    // Giá trị không có dữ liệu
};

struct TerrainGradient {
    double dz_de{0.0};               // Đạo hàm cao độ theo trục Đông-Tây ∂z/∂E
    double dz_dn{0.0};               // Đạo hàm cao độ theo trục Bắc-Nam ∂z/∂N
    double gradient_norm{0.0};       // Độ lớn Gradient: ||∇z|| = sqrt((∂z/∂E)^2 + (∂z/∂N)^2)
    double slope_rad{0.0};           // Góc dốc địa hình: arctan(||∇z||) [rad]
    double aspect_rad{0.0};          // Hướng dốc sườn núi (từ Bắc theo chiều kim đồng hồ) [rad]
    bool is_valid{false};
};

/**
 * @brief Bản đồ số độ cao DEM (Digital Elevation Model) phẳng cục bộ
 * Hỗ trợ nạp Memory-Mapped File (Zero-Copy), nạp từ RAM, nội suy song tuyến tính,
 * tính Gradient/Aspect và chuyển đổi tọa độ ENU <-> WGS84.
 */
class DemMap {
public:
    DemMap() noexcept;
    ~DemMap() noexcept;

    // Chuyển quyền sở hữu (Move semantics)
    DemMap(DemMap&& other) noexcept;
    DemMap& operator=(DemMap&& other) noexcept;

    // Không sao chép trực tiếp để quản lý an toàn file mapping
    DemMap(const DemMap& other);
    DemMap& operator=(const DemMap& other);

    /**
     * @brief Nạp tệp nhị phân .sdem từ đĩa (hỗ trợ Memory Mapping)
     */
    bool loadFromFile(const std::string& filepath, bool use_mmap = true) noexcept;

    /**
     * @brief Ghi bản đồ hiện tại ra tệp nhị phân .sdem kèm mã kiểm tra CRC32
     */
    bool saveToFile(const std::string& filepath) const noexcept;

    /**
     * @brief Nạp dữ liệu raster từ mảng bộ nhớ liên tục (Row-Major: từ Nam lên Bắc, từ Tây sang Đông)
     */
    bool loadFromMemory(const DemHeader& header, std::span<const float> elevation_data) noexcept;

    /**
     * @brief Tạo bản đồ DEM mô phỏng đỉnh núi nón (Synthetic Cone) phục vụ kiểm thử
     */
    bool createSyntheticCone(double width_m, double height_m, double cell_size_m,
                             double center_e_m, double center_n_m,
                             double peak_alt_m, double base_alt_m, double base_radius_m,
                             double origin_lat_rad = 0.366519, double origin_lon_rad = 1.8326) noexcept;

    /**
     * @brief Tạo bản đồ DEM mô phỏng sống núi nghiêng 1 chiều (Synthetic Ridge/Ramp)
     */
    bool createSyntheticRamp(double width_m, double height_m, double cell_size_m,
                             double min_alt_m, double max_alt_m,
                             double origin_lat_rad = 0.366519, double origin_lon_rad = 1.8326) noexcept;

    /**
     * @brief Nội suy song tuyến tính cao độ z_dem tại tọa độ thực (Easting, Northing)
     */
    [[nodiscard]] std::optional<double> getElevationBilinear(double easting_m, double northing_m) const noexcept;

    /**
     * @brief Tính Gradient địa hình, độ dốc slope và hướng sườn dốc aspect
     */
    [[nodiscard]] TerrainGradient getGradient(double easting_m, double northing_m) const noexcept;

    /**
     * @brief Kiểm tra địa hình có khả vi hành (slope <= max_slope_rad, mặc định 55 độ = 0.96 rad)
     */
    [[nodiscard]] bool isTraversable(double easting_m, double northing_m, double max_slope_rad = 0.96) const noexcept;

    /**
     * @brief Kiểm tra tọa độ có nằm trong phạm vi bản đồ
     */
    [[nodiscard]] bool isInside(double easting_m, double northing_m) const noexcept;

    /**
     * @brief Chuyển đổi tọa độ phẳng ENU sang trắc địa WGS84 (Local Flat-Earth Approximation)
     */
    [[nodiscard]] std::pair<double, double> enuToLatLon(double easting_m, double northing_m) const noexcept;

    /**
     * @brief Chuyển đổi trắc địa WGS84 sang tọa độ phẳng ENU
     */
    [[nodiscard]] std::pair<double, double> latLonToEnu(double lat_rad, double lon_rad) const noexcept;

    /**
     * @brief Đăng ký factory khởi tạo mmap từ composition root (e.g. platform)
     */
    static void registerMemoryMapFactory(platform::MemoryMapFactoryFn factory) noexcept;

    /**
     * @brief Nạp dữ liệu bản đồ trực tiếp từ một Memory-Mapped Buffer được inject
     */
    bool loadFromMemoryMap(std::unique_ptr<platform::IMemoryMappedBuffer> mmap_buf) noexcept;

    /**
     * @brief Nạp dữ liệu bản đồ từ luồng std::istream chuẩn C++
     */
    bool loadFromStream(std::istream& is) noexcept;

    [[nodiscard]] const DemHeader& getHeader() const noexcept {
        return header_;
    }

    [[nodiscard]] bool isLoaded() const noexcept {
        return (mmap_ptr_ != nullptr) || !elevation_grid_.empty();
    }

    [[nodiscard]] bool isMemoryMapped() const noexcept {
        return mmap_buffer_ != nullptr && mmap_buffer_->isValid();
    }

    void clear() noexcept;

private:
    [[nodiscard]] size_t getCellIndex(size_t col, size_t row) const noexcept {
        return row * header_.width_cells + col;
    }

    [[nodiscard]] float getRawElevation(size_t col, size_t row) const noexcept {
        const size_t idx = getCellIndex(col, row);
        if (mmap_ptr_ != nullptr) {
            return mmap_ptr_[idx];
        }
        if (idx < elevation_grid_.size()) {
            return elevation_grid_[idx];
        }
        return header_.nodata_value;
    }

    void unmap() noexcept;

    DemHeader header_{};
    std::vector<float> elevation_grid_{};

    std::unique_ptr<platform::IMemoryMappedBuffer> mmap_buffer_{nullptr};
    const float* mmap_ptr_{nullptr};

    static inline platform::MemoryMapFactoryFn s_mmap_factory_{nullptr};
};

} // namespace survival::environment
