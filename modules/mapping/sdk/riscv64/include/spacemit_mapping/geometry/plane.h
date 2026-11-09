// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_PLANE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_PLANE_H_

#include "spacemit_mapping/core/types.h"

namespace spacemit_mapping
{

class Plane
{
public:
    Plane() = default;
    // Normalize the supplied normal; offset is already in metres and stays unchanged.
    Plane(Vector3f normal, float offset_m);
    Plane(Vector3f normal, Vector3f point_m);
    // Preserve saved coefficients; reject normals outside eight float epsilons of unit length.
    static Plane from_normalized(Vector3f normal, float offset_m);
    const Vector3f& normal() const noexcept { return normal_; }
    float offset() const noexcept { return offset_; }
    float signed_distance(Vector3f point_m) const;
    // A vertical plane has no unique height: IEEE division produces Inf/NaN.
    float height_at_xy(float x_m, float y_m) const;

private:
    Vector3f normal_{1, 0, 0};
    float offset_ = 0;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_PLANE_H_
