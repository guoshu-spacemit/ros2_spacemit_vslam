// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_CAMERA_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_CAMERA_H_

#include "spacemit_mapping/core/types.h"

namespace spacemit_mapping
{

struct Pixel2f
{
    float u = 0;
    float v = 0;
};

struct Pixel2i
{
    int u = 0;
    int v = 0;
};

Pixel2f pixel_center(Pixel2i pixel);
// Floor image-plane coordinates, including negative values; invalid inputs throw.
Pixel2i pixel_index(Pixel2f pixel);

class PinholeCamera
{
public:
    PinholeCamera(int width, int height, float fx, float fy, float cx, float cy);
    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }
    float fx() const noexcept { return fx_; }
    float fy() const noexcept { return fy_; }
    float cx() const noexcept { return cx_; }
    float cy() const noexcept { return cy_; }
    // Positive camera Z; minimum_z is an optional inclusive near limit.
    // All failed projections leave the output unchanged.
    bool project(const Vector3f& point, Pixel2f* pixel, bool check_viewport = true,
        float minimum_z = 0) const;
    bool project(const Vector3f& point, Pixel2i* pixel, bool check_viewport = true,
        float minimum_z = 0) const;
    Vector3f unproject(Pixel2f pixel, float depth_z_m) const;
    Vector3f unproject(Pixel2i pixel, float depth_z_m) const;

private:
    int width_;
    int height_;
    float fx_, fy_, cx_, cy_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_CAMERA_H_
