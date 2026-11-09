// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_CORE_TYPES_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_CORE_TYPES_H_

#include <cstddef>
#include <cstdint>

namespace spacemit_mapping
{

struct Index3D
{
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    constexpr bool operator==(const Index3D& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const Index3D& other) const noexcept { return !(*this == other); }

    constexpr bool operator<(const Index3D& other) const noexcept
    {
        if (x != other.x)
            return x < other.x;
        if (y != other.y)
            return y < other.y;
        return z < other.z;
    }
};

struct Index3DHash
{
    std::size_t operator()(const Index3D& index) const noexcept;
};

struct Vector3f
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Rgb8
{
    std::uint8_t red = 0;
    std::uint8_t green = 0;
    std::uint8_t blue = 0;

    constexpr bool operator==(const Rgb8& other) const noexcept
    {
        return red == other.red && green == other.green && blue == other.blue;
    }
};

using TimestampMs = std::int64_t;

enum class Backend
{
    kAuto,
    kScalar,
    kRvv,
};

// Compile-target capability only. This is not a runtime CPU feature probe.
bool rvv_compiled() noexcept;

// Explicit RVV requests fail when unavailable. Auto falls back to scalar.
Backend resolve_backend(Backend requested, bool rvv_available);

// Mathematical floor for signed coordinates; divisor must be positive.
std::int32_t floor_div(std::int32_t value, std::int32_t divisor);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_CORE_TYPES_H_
