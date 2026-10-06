# CODE REVIEW — THƯ MỤC `platform/` (TẦNG TRỪU TƯỢNG HỆ ĐIỀU HÀNH)

**Ngày review:** 06/09/2026 (sau Sprint S2) | **Phạm vi:** `platform/src/` + `platform/include/survival_platform/`

---

## 1. TỔNG QUAN

Lớp mỏng trừu tượng hóa OS: `platform_memory_map` (Win32 `CreateFileMappingW` / POSIX `mmap`), `platform_file_system` (`FlushFileBuffers` / `fsync`), `platform_factory` (factory + `initializePlatformHooks`). Tuân thủ Invariant 3: core chỉ thấy interface PAL (`core/include/survival_core/platform/*.hpp`).

## 2. ĐIỂM MẠNH

- **Quản lý tài nguyên đúng**: mọi error path đều close handle đúng thứ tự (unmap → close mapping → close file); destructor RAII đầy đủ; POSIX path (`O_RDONLY`, `MAP_SHARED`, `munmap`/`close`) chuẩn.
- **Validate trước khi map**: size ≤ 0 bị chặn (`memory_map.cpp:101-105`), `PAGE_READONLY`/`FILE_MAP_READ` nhất quán, `isValid()` yêu cầu non-null + non-zero.
- **M15**: `initializePlatformHooks()` giờ **idempotent** (static guard) và được `survival_engine_create` gọi tự động — production có fsync/mmap thật mà không phụ thuộc caller.

## 3. LỖI ĐÃ FIX (S2/M14)

- `CreateFileA`/`CreateFileMappingA` (`platform_memory_map.cpp:88,107`) và `CreateFileA` (`platform_file_system.cpp:28`) — ANSI codepage fail với path có dấu tiếng Việt. Đã thay `CreateFileW`/`CreateFileMappingW` + helper `utf8ToWide` (MultiByteToWideChar CP_UTF8). Test: `DemMmapIoTest.VietnameseDiacriticPathLoads` với tên file `bản_đểm_thử.sdem` ✅.

## 4. VẤN ĐỀ CÒN TỒN ĐỌNG

1. **[LOW] Helper `utf8ToWide` bị nhân bản** ở 2 file (`platform_memory_map.cpp:24` và inline trong `platform_file_system.cpp`) — khuyến nghị gộp thành 1 helper dùng chung (export từ factory header trong `#ifdef _WIN32`).
2. **[LOW] Dependency hướng ngược tại điểm nối** — `survival_c_api.cpp` (thuộc core) gọi `initializePlatformHooks` từ platform: header thuần declaration nên không vi phạm Invariant 3 (không OS header vào core), nhưng về kiến trúc nên cân nhắc move c_api sang target riêng hoặc dùng function-pointer hook do consumer đăng ký nếu muốn core link độc lập hoàn toàn. Hiện tại: core CMake thêm PRIVATE include dir; symbol resolve ở tầng cuối (tests/desktop đều link platform) — hoạt động đúng.
3. **[LOW] `initializePlatformHooks` không đảo ngược được** — không có `shutdownPlatformHooks()`; static guard một chiều là đủ cho use-case hiện tại nhưng nên ghi rõ trong header.
4. **[INFO] Chưa có tầng Android/NDK** — factory chỉ có Win32 + POSIX; khi dựng khung mobile (v2.4) cần thêm nhánh AAssetManager/NDK file API.

## 5. KHUYẾN NGHỊ

- Gộp helper UTF-8 dùng chung; thêm unit test POSIX path (hiện chỉ test trên Windows).
- Khi làm mobile: thêm `createPlatformFileSystemAndroid` (fsync qua `ashmem`/FD), giữ contract `PlatformIoResult` không đổi.

**Điểm đánh giá chung: 8/10** — lớp mỏng, đúng nhiệm vụ, đã vá xong lỗi path Unicode; còn nợ gọn helper và tầng Android.
