// Interrupt-driven GPIO exposure timestamps and matching.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_CAMERA_SDK_GPIO_EXPOSURE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_CAMERA_SDK_GPIO_EXPOSURE_H_
#include "camera_sdk/exposure_matcher.h"
#include <mutex>
#include <thread>

namespace camera_sdk
{
// GPIO cdev v2 reader: kernel timestamps, interrupt-driven blocking wait.
// Never unexports GPIOs, modifies pinmux, or controls camera exposure.
class GpioExposure
{
public:
    GpioExposure(
        const std::string& chip, unsigned line, bool active_low, ExposureMatcher::Config config);
    ~GpioExposure();
    GpioExposure(const GpioExposure&) = delete;
    GpioExposure& operator=(const GpioExposure&) = delete;
    std::optional<ExposureMatcher::Exposure> match(ExposureMatcher::Exposure device);
    std::string status() const;
    void reset();
    static std::int64_t monotonic_ns();

private:
    void run() noexcept;
    void invalidate(const std::string& reason);
    ExposureMatcher::Config config_;
    ExposureMatcher matcher_;
    mutable std::mutex mutex_;
    std::deque<ExposureMatcher::Exposure> pairs_;
    std::thread worker_;
    int line_fd_{-1}, stop_fd_{-1};
    bool failed_{false};
    std::uint64_t discontinuities_{0};
};
} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_CAMERA_SDK_GPIO_EXPOSURE_H_
