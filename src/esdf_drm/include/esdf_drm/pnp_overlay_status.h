#ifndef PNP_OVERLAY_STATUS_H
#define PNP_OVERLAY_STATUS_H
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

namespace esdf_drm
{

struct PnpOverlayStatus
{
    bool available = false;
    bool pose_valid = false;
    bool attempted = false;
    bool success = false;
    bool inertial = false;
    bool imu_prediction = false;
    std::uint32_t streak = 0;
    float span_ms = 0.0F;
    std::uint32_t minimum_observations = 0;
    bool evaluated = false;
    std::uint32_t depth_valid = 0;
    std::uint32_t projected = 0;
    std::uint32_t unique = 0;
    std::uint32_t over_3px = 0;
    float rmse_px = std::numeric_limits<float>::quiet_NaN();
    bool weights_evaluated = false;
    std::uint32_t weight_count = 0;
    std::uint32_t weight_below_half = 0;
    float weight_mean = std::numeric_limits<float>::quiet_NaN();
    float weight_min = std::numeric_limits<float>::quiet_NaN();
    float imu_scale = std::numeric_limits<float>::quiet_NaN();
};

struct PnpOverlayText
{
    static constexpr std::size_t kLineCount = 7;
    std::array<std::string, kLineCount> lines;
    std::array<float, 3> color{0.8F, 0.8F, 0.8F};
};

inline PnpOverlayText describe_pnp(const PnpOverlayStatus& s, std::uint32_t input,
    std::uint32_t left_selected, std::uint32_t left_old, std::uint32_t age_limit)
{
    PnpOverlayText text;
    if (!s.available)
    {
        text.lines[0] = "DIAGNOSTICS OFF";
        return text;
    }
    char buffer[96];
    if (!s.pose_valid)
    {
        text.lines[0] = s.attempted ? "LOST - PNP FAIL" : "NO POSE";
        text.color = {1.0F, 0.25F, 0.2F};
    }
    else if (s.imu_prediction)
    {
        text.lines[0] = s.attempted ? "IMU ONLY - PNP FAIL" : "IMU ONLY - NO PNP";
        text.color = {1.0F, 0.25F, 0.2F};
    }
    else if (s.attempted && s.success)
    {
        text.lines[0] = s.inertial ? "PNP OK VIO" : "PNP OK VO";
        text.color = {0.2F, 1.0F, 0.4F};
    }
    else
        text.lines[0] = "INIT - NO PNP";

    if (s.evaluated)
        std::snprintf(buffer, sizeof(buffer), "IN %u ZOK %u U %u", input, s.depth_valid, s.unique);
    else
        std::snprintf(buffer, sizeof(buffer), "IN %u ZOK -- U --", input);
    text.lines[1] = buffer;
    if (s.evaluated && s.projected != 0 && std::isfinite(s.rmse_px))
        std::snprintf(
            buffer, sizeof(buffer), "ERR %.2fPX >3 %u/%u", s.rmse_px, s.over_3px, s.projected);
    else
        std::snprintf(buffer, sizeof(buffer), "ERR --PX >3 --");
    text.lines[2] = buffer;
    if (s.imu_prediction)
        std::snprintf(buffer, sizeof(buffer), "RUN %uF SPAN %.0fMS", s.streak, s.span_ms);
    else
        std::snprintf(
            buffer, sizeof(buffer), "LEFT %u OLD%u+ %u", left_selected, age_limit, left_old);
    text.lines[3] = buffer;

    // Explicit display-only observations, never a diagnosis or solver gate.
    text.lines[4] = "FIT IS NOT POSE ACCURACY";
    if (s.imu_prediction)
        text.lines[4] = "NO VISUAL POSE CORRECTION";
    else if (s.success && s.evaluated)
    {
        if (s.depth_valid < s.minimum_observations)
            text.lines[4] = "CHECK: ZOK < PNP MIN";
        else if (s.projected == 0 || !std::isfinite(s.rmse_px))
            text.lines[4] = "CHECK: NO PIXEL RESIDUAL";
        else if (s.rmse_px > 3.0F)
            text.lines[4] = "CHECK: RMS > 3PX";
        else if (s.projected < s.depth_valid)
            text.lines[4] = "CHECK: PROJECTION INVALID";
        if (text.lines[4].compare(0, 6, "CHECK:") == 0)
            text.color = {1.0F, 0.75F, 0.15F};
    }
    // Report robust attenuation separately from the configured IMU multiplier.
    // Never fabricate a visual/IMU percentage from unlike information blocks.
    const bool weights_valid = s.pose_valid && s.attempted && s.success && s.inertial &&
        s.weights_evaluated && s.weight_count != 0 && s.weight_below_half <= s.weight_count &&
        std::isfinite(s.weight_mean) && std::isfinite(s.weight_min) && s.weight_min >= 0.0F &&
        s.weight_min <= s.weight_mean && s.weight_mean <= 1.0F;
    if (weights_valid)
        std::snprintf(
            buffer, sizeof(buffer), "VIS W AVG %.2f MIN %.1E", s.weight_mean, s.weight_min);
    else
        std::snprintf(buffer, sizeof(buffer), "VIS W AVG -- MIN --");
    text.lines[5] = buffer;
    if (weights_valid)
        std::snprintf(buffer, sizeof(buffer), "W<0.5 %u/%u", s.weight_below_half, s.weight_count);
    else
        std::snprintf(buffer, sizeof(buffer), "W<0.5 --");
    text.lines[6] = buffer;
    if (s.attempted && s.inertial && std::isfinite(s.imu_scale) && s.imu_scale >= 0.0F)
        std::snprintf(buffer, sizeof(buffer), " IMU S %.1E", s.imu_scale);
    else
        std::snprintf(buffer, sizeof(buffer), " IMU S --");
    text.lines[6] += buffer;
    return text;
}

} // namespace esdf_drm

#endif // PNP_OVERLAY_STATUS_H
