#pragma once

#include "survival_core/platform/platform_memory_map.hpp"
#include "survival_core/platform/platform_file_system.hpp"
#include <memory>
#include <string>

namespace survival::platform {

/**
 * @brief Factory tạo Memory-Mapped Buffer theo từng OS (Win32 CreateFileMapping hoặc POSIX mmap)
 */
[[nodiscard]] std::unique_ptr<IMemoryMappedBuffer> createPlatformMemoryMap(const std::string& filepath) noexcept;

/**
 * @brief Factory tạo FileSystem hỗ trợ hardware sync theo từng OS (Win32 FlushFileBuffers hoặc POSIX fsync)
 */
[[nodiscard]] std::shared_ptr<IPlatformFileSystem> createPlatformFileSystem() noexcept;

/**
 * @brief Đăng ký các implementation của platform vào core composition hooks
 */
void initializePlatformHooks() noexcept;

} // namespace survival::platform
