#pragma once

#include <cstddef>
#include <memory>
#include <functional>
#include <string>

namespace survival::platform {

/**
 * @brief Interface trừu tượng cho vùng nhớ Memory-Mapped File (Pure C++20)
 * Tuân thủ nghiêm ngặt Invariant 3: Không chứa bất kỳ header hay kiểu dữ liệu của OS cụ thể.
 */
class IMemoryMappedBuffer {
public:
    virtual ~IMemoryMappedBuffer() = default;

    [[nodiscard]] virtual const float* data() const noexcept = 0;
    [[nodiscard]] virtual size_t sizeBytes() const noexcept = 0;
    [[nodiscard]] virtual bool isValid() const noexcept = 0;
};

using MemoryMapFactoryFn = std::function<std::unique_ptr<IMemoryMappedBuffer>(const std::string& filepath)>;

} // namespace survival::platform
