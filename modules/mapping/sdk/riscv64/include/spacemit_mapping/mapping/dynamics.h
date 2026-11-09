// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_DYNAMICS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_DYNAMICS_H_

#include "spacemit_mapping/geometry/lidar.h"
#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct DynamicsFrame
{
    std::size_t width = 0, height = 0;
    std::vector<std::uint8_t> mask;
    std::vector<Rgb8> overlay;
    std::vector<Vector3f> points;
};

// Owns every result; input image and history are read only during this call.
DynamicsFrame detect_dynamics(ImageView<float> depth, const PinholeCamera& camera,
    const RigidTransform& layer_from_camera, const FreespaceLayer& history);
DynamicsFrame detect_dynamics(ImageView<float> ranges, const Lidar& lidar,
    const RigidTransform& layer_from_sensor, const FreespaceLayer& history);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_DYNAMICS_H_
