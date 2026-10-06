#include "survival_platform/platform_factory.hpp"
#include "survival_core/environment/dem_map.hpp"
#include "survival_core/storage/local_blackbox.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#endif

namespace survival::platform {

namespace {

class PlatformFileSystemImpl final : public IPlatformFileSystem {
public:
    PlatformIoResult hardwareSync(const std::string& filepath) noexcept override {
        if (filepath.empty()) {
            return PlatformIoResult::kFileNotFound;
        }

#ifdef _WIN32
        // [M14] Wide API + UTF-8 cho path tiếng Việt
        const int wlen = MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, nullptr, 0);
        if (wlen <= 0) {
            return PlatformIoResult::kInvalidHandle;
        }
        std::wstring wide_path(static_cast<size_t>(wlen - 1), L'\0');
        MultiByteToWideChar(CP_UTF8, 0, filepath.c_str(), -1, wide_path.data(), wlen);

        HANDLE hFile = CreateFileW(
            wide_path.c_str(),
            GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );
        if (hFile == INVALID_HANDLE_VALUE) {
            return PlatformIoResult::kInvalidHandle;
        }

        const BOOL ok = FlushFileBuffers(hFile);
        CloseHandle(hFile);
        return ok ? PlatformIoResult::kSuccess : PlatformIoResult::kSyncFailed;
#else
        int fd = open(filepath.c_str(), O_WRONLY);
        if (fd < 0) {
            return PlatformIoResult::kInvalidHandle;
        }

        int res = fsync(fd);
        close(fd);
        return (res == 0) ? PlatformIoResult::kSuccess : PlatformIoResult::kSyncFailed;
#endif
    }
};

} // namespace

std::shared_ptr<IPlatformFileSystem> createPlatformFileSystem() noexcept {
    return std::make_shared<PlatformFileSystemImpl>();
}

void initializePlatformHooks() noexcept {
    // [M15] Idempotent: chỉ đăng ký một lần — được gọi từ survival_engine_create()
    // để production luôn có fsync/mmap mà không cần caller chủ động đăng ký.
    static bool hooks_initialized = false;
    if (hooks_initialized) {
        return;
    }
    hooks_initialized = true;

    // Đăng ký Memory Map Factory vào DemMap
    environment::DemMap::registerMemoryMapFactory([](const std::string& path) {
        return createPlatformMemoryMap(path);
    });

    // Đăng ký File System Factory vào LocalBlackbox
    storage::LocalBlackbox::registerPlatformFileSystem(createPlatformFileSystem());
}

} // namespace survival::platform
