# QUY CHUẨN LẬP TRÌNH & NGUYÊN TẮC PHÁT TRIỂN MÃ NGUỒN

**DỰ ÁN: OFF-GRID SURVIVAL ENGINE (C++20 CORE & RESEARCH HARNESS)**

---

### I. TIÊU CHUẨN BIÊN DỊCH & CỜ TRÌNH BIÊN DỊCH (COMPILER & TOOLCHAIN)

1. **Chuẩn ngôn ngữ:** Bắt buộc **C++20** trên toàn bộ module lõi (`core/`) và khung thực nghiệm (`harness/`).
2. **Cờ cảnh báo nghiêm ngặt (Strict Compiler Flags):**
* GCC / Clang: `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Werror`
* MSVC: `/W4 /WX /permissive-`


3. **Chính sách RTTI & Exception:**
* Thư mục `core/` biên dịch với cờ `-fno-rtti` và `-fno-exceptions` để đảm bảo tối ưu kích thước binary và tương thích tốt nhất với Android NDK / Vi điều khiển nhúng.
* Xử lý lỗi trong lõi sử dụng `std::expected` (hoặc `std::optional`) thay vì quăng ngoại lệ (`throw`).



---

### II. QUẢN LÝ BỘ NHỚ & CHÍNH SÁCH CẤP PHÁT (MEMORY & ZERO-ALLOCATION IN HOT-PATH)

Trong các chu trình tính toán thời gian thực (vòng lặp $50\text{Hz}$ của ESKF hoặc mỗi bước chân của Particle Filter), **tuyệt đối cấm cấp phát bộ nhớ động trên Heap** (`malloc`, `new`, `std::vector::push_back` gây reallocation).

```cpp
// ❌ SAI: Cấp phát động liên tục trong vòng lặp hạt
void updateParticles(const StepEstimate& step) {
    std::vector<Particle> temp_list; // Gây malloc/free mỗi chu kỳ
    for (int i = 0; i < 5000; ++i) {
        temp_list.push_back(propagate(particles_[i], step));
    }
    particles_ = temp_list;
}

//  ĐÚNG: Cấp phát trước bộ đệm tĩnh (Pre-allocated Arena/Buffer)
class ParticleFilter {
private:
    static constexpr size_t kMaxParticles = 5000;
    std::array<Particle, kMaxParticles> particles_;
    std::array<Particle, kMaxParticles> scratch_buffer_; // Bộ đệm hoán đổi
public:
    void updateParticles(const StepEstimate& step) noexcept;
};

```

1. **Pre-allocation:** Mọi cấu trúc dữ liệu (`std::vector`) phải được `reserve()` đầy đủ kích thước tối đa ngay tại hàm khởi tạo (Constructor).
2. **Con trỏ & Quyền sở hữu (Ownership):**
* Cấm sử dụng con trỏ thô (`T*`) để quản lý vòng đời bộ nhớ.
* Sử dụng `std::unique_ptr` cho quyền sở hữu độc quyền, `std::shared_ptr` bị hạn chế tối đa trừ khi thực sự cần chia sẻ tài nguyên phi chu kỳ.
* Dùng `std::span<const T>` hoặc `const T&` làm tham số truyền vào hàm để đọc dữ liệu mảng liên tục.



---

### III. AN TOÀN SỐ HỌC & BẢO TOÀN TÍNH XÁC ĐỊNH DƯƠNG (NUMERICAL INTEGRITY)

1. **Khởi tạo biến:** $100\%$ biến số thực (`float`, `double`) phải được khởi tạo giá trị cụ thể ngay khi khai báo. Không để tồn tại biến rác.
2. **Kiểm tra NaN / Infinity Guard:**
* Mọi dữ liệu cảm biến thô từ bên ngoài đẩy vào phải đi qua hàm kiểm tra:
```cpp
if (std::isnan(val) || std::isinf(val)) [[unlikely]] {
    // Kích hoạt Fallback / Log telemetry, không đưa vào ma trận
}

```




3. **Chuẩn hóa góc (Angle Normalization):**
* Mọi phép trừ góc trong Kalman Innovation hoặc tính sai số góc phương vị phải chuẩn hóa về $[-\pi, \pi]$ bằng `std::atan2(std::sin(delta), std::cos(delta))`. Cấm dùng phép chia lấy dư thô `% 360.0` trên số thực có dấu.


4. **Bảo toàn Hiệp phương sai:**
* Sau mỗi bước cập nhật ma trận $\mathbf{P}$, bắt buộc ép đối xứng: $\mathbf{P} = 0.5 \cdot (\mathbf{P} + \mathbf{P}^T)$.



---

### IV. QUY ƯỚC ĐẶT TÊN & CẤU TRÚC CODE (NAMING & FORMATTING)

Dự án tuân theo biến thể của **Google C++ Style Guide**:

| Thành phần | Quy tắc đặt tên | Ví dụ |
| --- | --- | --- |
| **Namespace** | Chữ thường, ngắn gọn | `survival::estimators`, `survival::math` |
| **Class / Struct / Concept** | PascalCase | `AdaptiveParametricModel`, `IEstimateSink` |
| **Function / Method** | camelCase | `estimateStepLength()`, `resampleSystematic()` |
| **Member Variable** | snake_case kèm dấu gạch dưới cuối | `sample_rate_hz_`, `particle_count_` |
| **Local Variable / Parameter** | snake_case | `delta_time_s`, `accel_norm` |
| **Constant / Enum Value** | Ký tự `k` + PascalCase | `kGravityStandard`, `kMaxParticles` |
| **Template Type** | Ký tự `T` hoặc PascalCase | `template <typename FloatType, size_t Capacity>` |

#### Quy ước File & Thư mục:

* Header file: `.hpp`, Source file: `.cpp`.
* Mỗi file header phải có `#pragma once` ở dòng đầu tiên.
* Header phải tự chứa đầy đủ (Self-contained): Nếu include file header vào một file `.cpp` trống thì bản thân nó phải biên dịch thành công.

---

### V. TIÊU CHUẨN C++20 & MODERN IDIOMS

1. **Const & Noexcept:**
* Mọi phương thức không làm thay đổi trạng thái class bắt buộc đánh dấu `const`.
* Mọi hàm toán học, xử lý số học không quăng lỗi phải đánh dấu `noexcept`.
* Sử dụng thuộc tính `[[nodiscard]]` cho toàn bộ các hàm trả về kết quả tính toán hoặc trạng thái thành công/thất bại (`StepEstimate`, `GNSResult`, `bool`).


2. **Concepts thay thế SFINAE:**
* Sử dụng C++20 Concepts để ràng buộc kiểu dữ liệu toán học:
```cpp
template <typename T>
concept NumericType = std::is_floating_point_v<T>;

template <NumericType T>
[[nodiscard]] constexpr T normalizeHeading(T rad) noexcept;

```




3. **Cấu trúc Dữ liệu Rõ ràng (Designated Initializers):**
* Khuyến khích khởi tạo struct theo tên trường của C++20 để tăng tính minh bạch:
```cpp
StepFeatures features{
    .cadence_hz = 1.85,
    .vertical_accel_p2p = 3.2,
    .vertical_accel_rms = 1.1,
    .terrain_slope_rad = 0.12,
    .step_period_var = 0.02,
    .user_leg_length_m = 0.88
};

```





---

### VI. TÍNH ĐỘC LẬP NỀN TẢNG (CORE ISOLATION LAW)

```text
[Platform / OS Layer] (Android NDK, POSIX, Windows, Embedded RTOS)
         │
         ▼ (Chỉ truyền mảng dữ liệu thô qua C-Interface / JNI)
┌────────────────────────────────────────────────────────┐
│                   MODULE CORE (C++20)                  │
│  - Hoàn toàn KHÔNG include <windows.h>, <android/*>    │
│  - Hoàn toàn KHÔNG tương tác trực tiếp với File System │
│  - Hoàn toàn KHÔNG gọi hàm Threading của OS            │
│  - Chỉ sử dụng C++ Standard Library & Eigen Headers    │
└────────────────────────────────────────────────────────┘

```

1. Module `core/` không được phép phụ thuộc vào bất kỳ API cụ thể nào của hệ điều hành.
2. Dữ liệu bản đồ DEM hoặc File log phải được trừu tượng hóa dưới dạng bộ nhớ đệm liên tục (`std::span<const uint8_t>`) hoặc interface `IStreamReader` trước khi nạp vào lõi.

---

### VII. TÍNH KHÁCH QUAN, TÁI LẬP & KIỂM THỬ (TESTING & DETERMINISM)

1. **Khả năng Tái lập Tuyệt đối (Deterministic PRNG):**
* Trong các thuật toán ngẫu nhiên (Particle Filter Propagation), cấm sử dụng `std::rand()` hoặc khởi tạo seed theo thời gian thực `time(nullptr)`.
* Toàn bộ bộ sinh số ngẫu nhiên phải sử dụng `std::mt19937_64` với Seed được truyền rõ ràng từ `ExperimentManifest`.


2. **Quy chuẩn Unit Test (GoogleTest):**
* Mọi công thức toán học (`JeanMeeus`, `Weinberg`, `ESKF Kinematics`) phải có file test tương ứng trong thư mục `tests/`.
* Phải bao phủ (cover) $100\%$ các trường hợp biên: Góc $0^\circ, 90^\circ, 180^\circ, 360^\circ$, tọa độ đi qua kinh tuyến $180^\circ$, bước chân đứng yên ($v = 0$), gia tốc trọng trường rơi tự do ($a = 0$).


3. **Đo lường Hiệu năng (Micro-benchmarking):**
* Mọi thay đổi thuật toán trong `core-navigation` phải chạy qua Google Benchmark. Thời gian xử lý của chu trình cập nhật bước chân không được vượt quá **$2.0\text{ ms}$** trên CPU đơn nhân tương đương ARM Cortex-A53.