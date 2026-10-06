#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <functional>

namespace survival::platform {

enum class PlatformIoResult : uint8_t {
    kSuccess = 0,
    kFileNotFound = 1,
    kAccessDenied = 2,
    kSyncFailed = 3,
    kInvalidHandle = 4,
    kUnknownError = 255
};

/**
 * @brief Interface trừu tượng cho thao tác đồng bộ phần cứng an toàn đĩa (Pure C++20)
 * Tuân thủ Invariant 3: Tách biệt hoàn toàn khỏi FlushFileBuffers/fsync của hệ điều hành.
 */
class IPlatformFileSystem {
public:
    virtual ~IPlatformFileSystem() = default;

    /**
     * @brief Đồng bộ bộ đệm file xuống phần cứng đĩa Flash
     */
    virtual PlatformIoResult hardwareSync(const std::string& filepath) noexcept = 0;
};

using FileSystemFactoryFn = std::function<std::shared_ptr<IPlatformFileSystem>()>;

} // namespace survival::platform
