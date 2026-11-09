// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_LIDAR_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_LIDAR_H_

#include "spacemit_mapping/geometry/pointcloud.h"

namespace spacemit_mapping
{

class Lidar
{
public:
    Lidar(int azimuth_divisions, int elevation_divisions, float minimum_range_m,
        float vertical_fov_rad);
    Lidar(int azimuth_divisions, int elevation_divisions, float minimum_range_m,
        float angle_below_zero_rad, float angle_above_zero_rad);
    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }
    std::size_t size() const noexcept;
    float minimum_range() const noexcept { return minimum_range_; }
    float vertical_fov() const noexcept { return vertical_fov_; }
    float start_polar_angle() const noexcept { return start_polar_; }
    float angle_below_zero() const noexcept { return below_; }
    float angle_above_zero() const noexcept { return above_; }
    bool valid_range(Vector3f point) const;
    float range(Vector3f point) const;
    bool project(Vector3f point, Pixel2f* pixel, bool check_viewport = true) const;
    bool project(Vector3f point, Pixel2i* pixel, bool check_viewport = true) const;
    Vector3f ray(Pixel2f pixel) const;
    Vector3f ray(Pixel2i pixel) const;
    Vector3f unproject(Pixel2f pixel, float range_m) const;
    Vector3f unproject(Pixel2i pixel, float range_m) const;
    float linear_max_difference_vox() const noexcept { return linear_limit_; }
    float nearest_max_off_ray_vox() const noexcept { return nearest_limit_; }
    void set_linear_max_difference_vox(float value);
    void set_nearest_max_off_ray_vox(float value);
    // Bilinear then closest-pixel fallback. A successful fallback can yield NaN/0.
    // Failure leaves both outputs unchanged; malformed input throws.
    bool sample_range(ImageView<float> image, Pixel2f pixel, Vector3f voxel_center,
        float voxel_size_m, float* depth, Pixel2i* closest = nullptr) const;

private:
    int width_, height_;
    float minimum_range_, minimum_range_squared_, below_, above_, vertical_fov_;
    float elevation_step_, azimuth_step_, elevation_scale_, azimuth_scale_;
    float start_polar_, start_azimuth_;
    float linear_limit_ = 2;
    float nearest_limit_ = 0.5F;
};

struct ScanMotion
{
    RigidTransform end_pose;
    TimestampMs duration_ms = 0;
};

// Ranges are in the scan-start frame. Nearest return wins; empty cells are zero.
// Motion compensation requires per-point timestamps in [0,duration_ms].
std::vector<float> range_image_from_pointcloud(const Pointcloud& input,
    const RigidTransform& start_pose, const Lidar& lidar,
    const std::optional<ScanMotion>& motion = std::nullopt);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_LIDAR_H_
