#include "survival_core/environment/dem_map.hpp"
#include "survival_core/math/crc32.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <cstring>

namespace survival::environment {

namespace {
// [H4] Trần cứng số ô DEM (64 triệu ô = 256 MB payload float) — chặn bad_alloc/length_error
// từ header giả mạo/hỏng bên trong hàm noexcept (throw ở đây = std::terminate).
constexpr uint64_t kMaxDemCells = 64ull * 1024ull * 1024ull;

/**
 * @brief Kiểm tra tính hợp lệ header .sdem TRƯỚC KHI cấp phát bộ đệm payload.
 * @param available_payload_bytes số byte thực tế còn lại sau header (từ stream/mmap/span).
 */
bool validateSdemHeader(const SdemFileHeader& hdr, uint64_t available_payload_bytes) noexcept {
    if (hdr.magic != 0x5344454D || hdr.version != 1) {
        return false;
    }
    if (hdr.width_cells == 0 || hdr.height_cells == 0) {
        return false;
    }
    // NaN <= 0.0 là false nên phải isfinite tường minh
    if (!(hdr.cell_size_m > 0.0F) || !std::isfinite(hdr.cell_size_m)) {
        return false;
    }
    const uint64_t cells = static_cast<uint64_t>(hdr.width_cells) * static_cast<uint64_t>(hdr.height_cells);
    if (cells > kMaxDemCells) {
        return false;
    }
    const uint64_t payload_bytes = cells * sizeof(float);
    if (payload_bytes > available_payload_bytes) {
        return false;  // File cụt / khai báo dims vượt dung lượng thực tế
    }
    return true;
}
} // namespace

void DemMap::registerMemoryMapFactory(platform::MemoryMapFactoryFn factory) noexcept {
    s_mmap_factory_ = std::move(factory);
}

DemMap::DemMap() noexcept = default;

DemMap::~DemMap() noexcept {
    unmap();
}

DemMap::DemMap(DemMap&& other) noexcept
    : header_(other.header_)
    , elevation_grid_(std::move(other.elevation_grid_))
    , mmap_buffer_(std::move(other.mmap_buffer_))
    , mmap_ptr_(other.mmap_ptr_)
{
    other.mmap_ptr_ = nullptr;
    other.header_ = DemHeader{};
}

DemMap& DemMap::operator=(DemMap&& other) noexcept {
    if (this != &other) {
        unmap();
        header_ = other.header_;
        elevation_grid_ = std::move(other.elevation_grid_);
        mmap_buffer_ = std::move(other.mmap_buffer_);
        mmap_ptr_ = other.mmap_ptr_;

        other.mmap_ptr_ = nullptr;
        other.header_ = DemHeader{};
    }
    return *this;
}

DemMap::DemMap(const DemMap& other)
    : header_(other.header_)
    , elevation_grid_(other.elevation_grid_)
{
    if (other.isMemoryMapped() && other.mmap_ptr_ != nullptr) {
        const size_t total_elements = header_.width_cells * header_.height_cells;
        elevation_grid_.assign(other.mmap_ptr_, other.mmap_ptr_ + total_elements);
    }
}

DemMap& DemMap::operator=(const DemMap& other) {
    if (this != &other) {
        unmap();
        header_ = other.header_;
        elevation_grid_ = other.elevation_grid_;
        if (other.isMemoryMapped() && other.mmap_ptr_ != nullptr) {
            const size_t total_elements = header_.width_cells * header_.height_cells;
            elevation_grid_.assign(other.mmap_ptr_, other.mmap_ptr_ + total_elements);
        }
    }
    return *this;
}

void DemMap::unmap() noexcept {
    mmap_buffer_.reset();
    mmap_ptr_ = nullptr;
}

void DemMap::clear() noexcept {
    unmap();
    elevation_grid_.clear();
    header_ = DemHeader{};
}

bool DemMap::loadFromMemory(const DemHeader& header, std::span<const float> elevation_data) noexcept {
    if (header.width_cells == 0 || header.height_cells == 0) {
        return false;
    }
    // NaN <= 0.0 là false -> phải isfinite tường minh [H4]
    if (!(header.cell_size_m > 0.0) || !std::isfinite(header.cell_size_m)) {
        return false;
    }

    const size_t expected_size = header.width_cells * header.height_cells;
    if (elevation_data.size() < expected_size) {
        return false;
    }

    unmap();
    header_ = header;
    elevation_grid_.assign(elevation_data.begin(),
                           elevation_data.begin() + static_cast<std::ptrdiff_t>(expected_size));
    return true;
}

bool DemMap::saveToFile(const std::string& filepath) const noexcept {
    if (!isLoaded()) {
        return false;
    }

    std::ofstream out(filepath, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }

    const size_t total_elements = header_.width_cells * header_.height_cells;
    const size_t payload_bytes = total_elements * sizeof(float);

    const float* data_ptr = mmap_ptr_ != nullptr ? mmap_ptr_ : elevation_grid_.data();
    if (data_ptr == nullptr) {
        return false;
    }

    const auto byte_span = std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(data_ptr), payload_bytes);
    const uint32_t crc = math::Crc32::calculate(byte_span);

    SdemFileHeader file_hdr{};
    file_hdr.magic = 0x5344454D;
    file_hdr.version = 1;
    file_hdr.header_size = 64;
    file_hdr.origin_easting_m = header_.origin_easting_m;
    file_hdr.origin_northing_m = header_.origin_northing_m;
    file_hdr.origin_lat_rad = header_.origin_lat_rad;
    file_hdr.origin_lon_rad = header_.origin_lon_rad;
    file_hdr.cell_size_m = header_.cell_size_m;
    file_hdr.width_cells = static_cast<uint32_t>(header_.width_cells);
    file_hdr.height_cells = static_cast<uint32_t>(header_.height_cells);
    file_hdr.nodata_value = header_.nodata_value;
    file_hdr.payload_crc32 = crc;

    out.write(reinterpret_cast<const char*>(&file_hdr), sizeof(SdemFileHeader));
    out.write(reinterpret_cast<const char*>(data_ptr), static_cast<std::streamsize>(payload_bytes));
    return out.good();
}

bool DemMap::loadFromMemoryMap(std::unique_ptr<platform::IMemoryMappedBuffer> mmap_buf) noexcept {
    if (!mmap_buf || !mmap_buf->isValid()) {
        return false;
    }

    if (mmap_buf->sizeBytes() < sizeof(SdemFileHeader)) {
        return false;
    }

    const auto* file_hdr = reinterpret_cast<const SdemFileHeader*>(mmap_buf->data());
    // [H4] Validate đầy đủ dims/cell_size/khớp dung lượng TRƯỚC khi dùng payload
    if (!validateSdemHeader(*file_hdr,
                            static_cast<uint64_t>(mmap_buf->sizeBytes()) - sizeof(SdemFileHeader))) {
        return false;
    }

    const size_t total_elements = static_cast<size_t>(file_hdr->width_cells) * file_hdr->height_cells;
    const size_t expected_payload_bytes = total_elements * sizeof(float);
    if (mmap_buf->sizeBytes() < sizeof(SdemFileHeader) + expected_payload_bytes) {
        return false;
    }

    const auto* payload_start = reinterpret_cast<const uint8_t*>(mmap_buf->data()) + sizeof(SdemFileHeader);
    const uint32_t calc_crc = math::Crc32::calculate(std::span<const uint8_t>(payload_start, expected_payload_bytes));
    if (calc_crc != file_hdr->payload_crc32) {
        return false;
    }

    unmap();
    elevation_grid_.clear();

    header_.origin_easting_m = file_hdr->origin_easting_m;
    header_.origin_northing_m = file_hdr->origin_northing_m;
    header_.origin_lat_rad = file_hdr->origin_lat_rad;
    header_.origin_lon_rad = file_hdr->origin_lon_rad;
    header_.cell_size_m = file_hdr->cell_size_m;
    header_.width_cells = file_hdr->width_cells;
    header_.height_cells = file_hdr->height_cells;
    header_.nodata_value = file_hdr->nodata_value;

    mmap_ptr_ = reinterpret_cast<const float*>(payload_start);
    mmap_buffer_ = std::move(mmap_buf);
    return true;
}

bool DemMap::loadFromStream(std::istream& is) noexcept {
    if (!is.good()) {
        return false;
    }

    SdemFileHeader file_hdr{};
    is.read(reinterpret_cast<char*>(&file_hdr), sizeof(SdemFileHeader));
    if (!is.good() || file_hdr.magic != 0x5344454D || file_hdr.version != 1) {
        return false;
    }

    // [H4] Xác định dung lượng còn lại của stream TRƯỚC KHI cấp phát — stream không
    // seekable (tellg < 0) bị từ chối; chặn bad_alloc trong noexcept (std::terminate).
    is.seekg(0, std::ios::end);
    const auto end_pos = is.tellg();
    is.seekg(static_cast<std::streamoff>(sizeof(SdemFileHeader)), std::ios::beg);
    if (end_pos < 0) {
        return false;
    }
    const auto available_payload = static_cast<uint64_t>(end_pos) - sizeof(SdemFileHeader);
    if (!validateSdemHeader(file_hdr, available_payload)) {
        return false;
    }

    const size_t total_elements = static_cast<size_t>(file_hdr.width_cells) * file_hdr.height_cells;
    const size_t payload_bytes = total_elements * sizeof(float);

    std::vector<float> grid(total_elements);
    is.read(reinterpret_cast<char*>(grid.data()), static_cast<std::streamsize>(payload_bytes));
    if (!is.good()) {
        return false;
    }

    const auto byte_span = std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(grid.data()), payload_bytes);
    if (math::Crc32::calculate(byte_span) != file_hdr.payload_crc32) {
        return false;
    }

    unmap();
    header_.origin_easting_m = file_hdr.origin_easting_m;
    header_.origin_northing_m = file_hdr.origin_northing_m;
    header_.origin_lat_rad = file_hdr.origin_lat_rad;
    header_.origin_lon_rad = file_hdr.origin_lon_rad;
    header_.cell_size_m = file_hdr.cell_size_m;
    header_.width_cells = file_hdr.width_cells;
    header_.height_cells = file_hdr.height_cells;
    header_.nodata_value = file_hdr.nodata_value;

    elevation_grid_ = std::move(grid);
    return true;
}

bool DemMap::loadFromFile(const std::string& filepath, bool use_mmap) noexcept {
    if (use_mmap && s_mmap_factory_ != nullptr) {
        auto mmap_buf = s_mmap_factory_(filepath);
        if (mmap_buf && mmap_buf->isValid()) {
            return loadFromMemoryMap(std::move(mmap_buf));
        }
    }

    std::ifstream in(filepath, std::ios::binary);
    if (!in.is_open()) {
        return false;
    }
    return loadFromStream(in);
}

bool DemMap::createSyntheticCone(double width_m, double height_m, double cell_size_m,
                                 double center_e_m, double center_n_m,
                                 double peak_alt_m, double base_alt_m, double base_radius_m,
                                 double origin_lat_rad, double origin_lon_rad) noexcept {
    if (width_m <= 0.0 || height_m <= 0.0 || cell_size_m <= 0.0 || base_radius_m <= 0.0) {
        return false;
    }

    unmap();

    header_.origin_easting_m = 0.0;
    header_.origin_northing_m = 0.0;
    header_.origin_lat_rad = origin_lat_rad;
    header_.origin_lon_rad = origin_lon_rad;
    header_.cell_size_m = cell_size_m;
    header_.width_cells = static_cast<size_t>(std::ceil(width_m / cell_size_m));
    header_.height_cells = static_cast<size_t>(std::ceil(height_m / cell_size_m));
    header_.nodata_value = -9999.0F;

    const size_t total_cells = header_.width_cells * header_.height_cells;
    elevation_grid_.resize(total_cells);

    const double alt_span = peak_alt_m - base_alt_m;

    for (size_t row = 0; row < header_.height_cells; ++row) {
        const double n_pos = static_cast<double>(row) * cell_size_m;
        for (size_t col = 0; col < header_.width_cells; ++col) {
            const double e_pos = static_cast<double>(col) * cell_size_m;

            const double de = e_pos - center_e_m;
            const double dn = n_pos - center_n_m;
            const double dist = std::sqrt(de * de + dn * dn);

            float alt = static_cast<float>(base_alt_m);
            if (dist < base_radius_m) {
                const double r_norm = dist / base_radius_m;
                alt = static_cast<float>(base_alt_m + alt_span * (1.0 - r_norm));
            }

            elevation_grid_[getCellIndex(col, row)] = alt;
        }
    }

    return true;
}

bool DemMap::createSyntheticRamp(double width_m, double height_m, double cell_size_m,
                                 double min_alt_m, double max_alt_m,
                                 double origin_lat_rad, double origin_lon_rad) noexcept {
    if (width_m <= 0.0 || height_m <= 0.0 || cell_size_m <= 0.0) {
        return false;
    }

    unmap();

    header_.origin_easting_m = 0.0;
    header_.origin_northing_m = 0.0;
    header_.origin_lat_rad = origin_lat_rad;
    header_.origin_lon_rad = origin_lon_rad;
    header_.cell_size_m = cell_size_m;
    header_.width_cells = static_cast<size_t>(std::ceil(width_m / cell_size_m));
    header_.height_cells = static_cast<size_t>(std::ceil(height_m / cell_size_m));
    header_.nodata_value = -9999.0F;

    const size_t total_cells = header_.width_cells * header_.height_cells;
    elevation_grid_.resize(total_cells);

    const double alt_span = max_alt_m - min_alt_m;

    for (size_t row = 0; row < header_.height_cells; ++row) {
        // Guard chia-zero khi lưới chỉ có 1 hàng (height_cells - 1 == 0 -> 0/0 = NaN)
        const double progress_n = (header_.height_cells > 1)
            ? static_cast<double>(row) / static_cast<double>(header_.height_cells - 1)
            : 0.0;
        const float row_alt = static_cast<float>(min_alt_m + alt_span * progress_n);

        for (size_t col = 0; col < header_.width_cells; ++col) {
            elevation_grid_[getCellIndex(col, row)] = row_alt;
        }
    }

    return true;
}

bool DemMap::isInside(double easting_m, double northing_m) const noexcept {
    if (!isLoaded()) {
        return false;
    }

    const double min_e = header_.origin_easting_m;
    const double max_e = min_e + static_cast<double>(header_.width_cells) * header_.cell_size_m;
    const double min_n = header_.origin_northing_m;
    const double max_n = min_n + static_cast<double>(header_.height_cells) * header_.cell_size_m;

    return (easting_m >= min_e && easting_m <= max_e && northing_m >= min_n && northing_m <= max_n);
}

std::optional<double> DemMap::getElevationBilinear(double easting_m, double northing_m) const noexcept {
    if (!isInside(easting_m, northing_m)) {
        return std::nullopt;
    }

    const double norm_x = (easting_m - header_.origin_easting_m) / header_.cell_size_m;
    const double norm_y = (northing_m - header_.origin_northing_m) / header_.cell_size_m;

    const auto col0 = static_cast<int64_t>(std::floor(norm_x));
    const auto row0 = static_cast<int64_t>(std::floor(norm_y));
    const int64_t col1 = col0 + 1;
    const int64_t row1 = row0 + 1;

    const double u = norm_x - static_cast<double>(col0);
    const double v = norm_y - static_cast<double>(row0);

    const auto c0_clamped = static_cast<size_t>(std::clamp<int64_t>(col0, 0, static_cast<int64_t>(header_.width_cells - 1)));
    const auto c1_clamped = static_cast<size_t>(std::clamp<int64_t>(col1, 0, static_cast<int64_t>(header_.width_cells - 1)));
    const auto r0_clamped = static_cast<size_t>(std::clamp<int64_t>(row0, 0, static_cast<int64_t>(header_.height_cells - 1)));
    const auto r1_clamped = static_cast<size_t>(std::clamp<int64_t>(row1, 0, static_cast<int64_t>(header_.height_cells - 1)));

    const float z00 = getRawElevation(c0_clamped, r0_clamped);
    const float z10 = getRawElevation(c1_clamped, r0_clamped);
    const float z01 = getRawElevation(c0_clamped, r1_clamped);
    const float z11 = getRawElevation(c1_clamped, r1_clamped);

    // [H4] NaN phải bị coi như nodata (NaN <= nodata là false nên lọt qua đường cũ)
    const auto is_usable = [this](float z) noexcept {
        return !std::isnan(z) && z > header_.nodata_value;
    };
    if (!is_usable(z00) || !is_usable(z10) || !is_usable(z01) || !is_usable(z11)) {
        return std::nullopt;
    }

    const double z = (1.0 - u) * (1.0 - v) * z00 +
                     u * (1.0 - v) * z10 +
                     (1.0 - u) * v * z01 +
                     u * v * z11;

    return z;
}

TerrainGradient DemMap::getGradient(double easting_m, double northing_m) const noexcept {
    TerrainGradient grad{};
    if (!isInside(easting_m, northing_m)) {
        return grad;
    }

    const double ds = header_.cell_size_m;
    const auto z_east = getElevationBilinear(easting_m + ds, northing_m);
    const auto z_west = getElevationBilinear(easting_m - ds, northing_m);
    const auto z_north = getElevationBilinear(easting_m, northing_m + ds);
    const auto z_south = getElevationBilinear(easting_m, northing_m - ds);

    if (!z_east || !z_west || !z_north || !z_south) {
        return grad;
    }

    grad.dz_de = (*z_east - *z_west) / (2.0 * ds);
    grad.dz_dn = (*z_north - *z_south) / (2.0 * ds);

    grad.gradient_norm = std::sqrt(grad.dz_de * grad.dz_de + grad.dz_dn * grad.dz_dn);
    grad.slope_rad = std::atan(grad.gradient_norm);

    constexpr double kMinSlopeThreshold = 0.005;
    if (grad.gradient_norm > kMinSlopeThreshold) {
        double aspect = std::atan2(-grad.dz_de, -grad.dz_dn);
        if (aspect < 0.0) {
            aspect += kTwoPi;
        }
        grad.aspect_rad = aspect;
    } else {
        grad.aspect_rad = 0.0;
    }

    grad.is_valid = true;
    return grad;
}

bool DemMap::isTraversable(double easting_m, double northing_m, double max_slope_rad) const noexcept {
    const auto grad = getGradient(easting_m, northing_m);
    if (!grad.is_valid) {
        return false;
    }
    return grad.slope_rad <= max_slope_rad;
}

std::pair<double, double> DemMap::enuToLatLon(double easting_m, double northing_m) const noexcept {
    const double delta_lat_rad = northing_m / kEarthRadiusMeanM;
    const double lat_rad = header_.origin_lat_rad + delta_lat_rad;

    const double cos_lat = std::max(0.01, std::cos(header_.origin_lat_rad));
    const double delta_lon_rad = easting_m / (kEarthRadiusMeanM * cos_lat);
    const double lon_rad = header_.origin_lon_rad + delta_lon_rad;

    return {lat_rad, lon_rad};
}

std::pair<double, double> DemMap::latLonToEnu(double lat_rad, double lon_rad) const noexcept {
    const double delta_lat_rad = lat_rad - header_.origin_lat_rad;
    const double northing_m = delta_lat_rad * kEarthRadiusMeanM;

    const double delta_lon_rad = lon_rad - header_.origin_lon_rad;
    const double cos_lat = std::max(0.01, std::cos(header_.origin_lat_rad));
    const double easting_m = delta_lon_rad * (kEarthRadiusMeanM * cos_lat);

    return {easting_m, northing_m};
}

} // namespace survival::environment
