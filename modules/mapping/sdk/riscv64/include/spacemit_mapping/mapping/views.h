// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_VIEWS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_VIEWS_H_

#include "spacemit_mapping/geometry/lidar.h"

#include <optional>

namespace spacemit_mapping
{

// Calibration/pose values are owned; image storage is borrowed for the call.
struct CameraMapView
{
    ImageView<float> depth;
    PinholeCamera camera;
    RigidTransform layer_from_camera;
    std::optional<float> maximum_distance_m;
    std::optional<float> truncation_m;
};

struct LidarMapView
{
    ImageView<float> ranges;
    Lidar lidar;
    RigidTransform layer_from_sensor;
    std::optional<float> maximum_distance_m;
    std::optional<float> truncation_m;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_VIEWS_H_
