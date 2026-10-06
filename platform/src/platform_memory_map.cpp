#include "survival_platform/platform_factory.hpp"
#include "survival_core/environment/dem_map.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace survival::platform {

namespace {

#ifdef _WIN32
// [M14] Chuyển UTF-8 -> UTF-16 cho Win32 Wide API: path có dấu tiếng Việt
// (CreateFileA theo ANSI codepage sẽ fail mở file)
[[nodiscard]] std::wstring utf8ToWide(const std::string& utf8) noexcept {
    if (utf8.empty()) {
        return std::wstring();
    }
    const int len = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (len <= 0) {
        return std::wstring();
    }
    std::wstring wide(static_cast<size_t>(len - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, wide.data(), len);
    return wide;
}
#endif

#ifdef _WIN32
class Win32MemoryMappedBuffer final : public IMemoryMappedBuffer {
public:
    Win32MemoryMappedBuffer(HANDLE file_handle, HANDLE map_handle, const float* data, size_t size_bytes) noexcept
        : file_handle_(file_handle)
        , map_handle_(map_handle)
        , data_(data)
        , size_bytes_(size_bytes)
    {
    }

    ~Win32MemoryMappedBuffer() noexcept override {
        if (data_ != nullptr) {
            UnmapViewOfFile(data_);
        }
        if (map_handle_ != nullptr) {
            CloseHandle(map_handle_);
        }
        if (file_handle_ != nullptr && file_handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(file_handle_);
        }
    }

    [[nodiscard]] const float* data() const noexcept override { return data_; }
    [[nodiscard]] size_t sizeBytes() const noexcept override { return size_bytes_; }
    [[nodiscard]] bool isValid() const noexcept override { return data_ != nullptr && size_bytes_ > 0; }

private:
    HANDLE file_handle_{INVALID_HANDLE_VALUE};
    HANDLE map_handle_{nullptr};
    const float* data_{nullptr};
    size_t size_bytes_{0};
};
#else
class PosixMemoryMappedBuffer final : public IMemoryMappedBuffer {
public:
    PosixMemoryMappedBuffer(int fd, const float* data, size_t size_bytes) noexcept
        : fd_(fd)
        , data_(data)
        , size_bytes_(size_bytes)
    {
    }

    ~PosixMemoryMappedBuffer() noexcept override {
        if (data_ != nullptr && size_bytes_ > 0) {
            munmap(const_cast<float*>(data_), size_bytes_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    [[nodiscard]] const float* data() const noexcept override { return data_; }
    [[nodiscard]] size_t sizeBytes() const noexcept override { return size_bytes_; }
    [[nodiscard]] bool isValid() const noexcept override { return data_ != nullptr && size_bytes_ > 0; }

private:
    int fd_{-1};
    const float* data_{nullptr};
    size_t size_bytes_{0};
};
#endif

} // namespace

std::unique_ptr<IMemoryMappedBuffer> createPlatformMemoryMap(const std::string& filepath) noexcept {
#ifdef _WIN32
    HANDLE file_handle = CreateFileW(
        utf8ToWide(filepath).c_str(),
        GENERIC_READ,
        FILE_SHARE_READ,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_READONLY | FILE_FLAG_SEQUENTIAL_SCAN,
        nullptr
    );
    if (file_handle == INVALID_HANDLE_VALUE) {
        return nullptr;
    }

    LARGE_INTEGER file_size;
    if (!GetFileSizeEx(file_handle, &file_size) || file_size.QuadPart <= 0) {
        CloseHandle(file_handle);
        return nullptr;
    }

    HANDLE map_handle = CreateFileMappingW(file_handle, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (map_handle == nullptr) {
        CloseHandle(file_handle);
        return nullptr;
    }

    const void* view_ptr = MapViewOfFile(map_handle, FILE_MAP_READ, 0, 0, 0);
    if (view_ptr == nullptr) {
        CloseHandle(map_handle);
        CloseHandle(file_handle);
        return nullptr;
    }

    return std::make_unique<Win32MemoryMappedBuffer>(
        file_handle,
        map_handle,
        static_cast<const float*>(view_ptr),
        static_cast<size_t>(file_size.QuadPart)
    );
#else
    int fd = open(filepath.c_str(), O_RDONLY);
    if (fd < 0) {
        return nullptr;
    }

    struct stat st;
    if (fstat(fd, &st) < 0 || st.st_size <= 0) {
        close(fd);
        return nullptr;
    }

    void* addr = mmap(nullptr, static_cast<size_t>(st.st_size), PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        close(fd);
        return nullptr;
    }

    return std::make_unique<PosixMemoryMappedBuffer>(
        fd,
        static_cast<const float*>(addr),
        static_cast<size_t>(st.st_size)
    );
#endif
}

} // namespace survival::platform
