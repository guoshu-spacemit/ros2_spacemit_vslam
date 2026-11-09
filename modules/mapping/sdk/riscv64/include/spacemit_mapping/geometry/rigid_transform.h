// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_RIGID_TRANSFORM_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_RIGID_TRANSFORM_H_

#include "spacemit_mapping/core/types.h"

#include <array>

namespace spacemit_mapping
{

// p_out = rotation * p_in + translation. Rotation is row-major, orthonormal,
// and right-handed. Translation uses metres. Default construction is identity.
struct RigidTransform
{
    std::array<float, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    Vector3f translation{};
};

Vector3f transform_point(const RigidTransform& transform, const Vector3f& point);
Vector3f inverse_transform_point(const RigidTransform& transform, const Vector3f& point);
RigidTransform inverse_rigid(const RigidTransform& transform);
RigidTransform compose(const RigidTransform& out_from_mid, const RigidTransform& mid_from_in);
// Shortest quaternion arc and linear translation. Alpha must be finite in [0,1].
RigidTransform interpolate_transform(
    const RigidTransform& begin, const RigidTransform& end, float alpha);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_RIGID_TRANSFORM_H_
