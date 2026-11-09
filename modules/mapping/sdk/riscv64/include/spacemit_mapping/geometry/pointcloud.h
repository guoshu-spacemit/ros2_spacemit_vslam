// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_POINTCLOUD_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_POINTCLOUD_H_

#include "spacemit_mapping/core/image_view.h"
#include "spacemit_mapping/geometry/camera.h"
#include "spacemit_mapping/geometry/rigid_transform.h"

#include <limits>
#include <optional>
#include <vector>

namespace spacemit_mapping
{

// Owning, contiguous points with optional one-to-one scan-relative timestamps.
// Copies own their data; resizing keeps timestamps aligned. Mutable point access
// cannot resize the arrays. File adapters belong to map_io.
class Pointcloud
{
public:
    explicit Pointcloud(std::size_t size = 0, bool with_timestamps = false);
    std::size_t size() const noexcept { return points_.size(); }
    bool empty() const noexcept { return points_.empty(); }
    Vector3f& point(std::size_t i) { return points_.at(i); }
    const Vector3f& point(std::size_t i) const { return points_.at(i); }
    const std::vector<Vector3f>& points() const noexcept { return points_; }
    const std::optional<std::vector<TimestampMs>>& timestamps() const noexcept
    {
        return timestamps_;
    }
    void resize(std::size_t size);
    void reserve(std::size_t capacity);
    void append(Vector3f point);
    void assign_points(const std::vector<Vector3f>& points);
    void set_timestamps(const std::vector<TimestampMs>& timestamps);
    void clear_timestamps() noexcept { timestamps_.reset(); }

private:
    std::vector<Vector3f> points_;
    std::optional<std::vector<TimestampMs>> timestamps_;
};

// Deliberately returns points only; timestamps describe the input scan frame.
Pointcloud transform_pointcloud(const RigidTransform& transform, const Pointcloud& input);
// Row-major compact output; skip nonfinite/nonpositive depths and depths > maximum.
Pointcloud pointcloud_from_depth(ImageView<float> depth, const PinholeCamera& camera,
    float maximum_depth_m = std::numeric_limits<float>::infinity());

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_POINTCLOUD_H_
