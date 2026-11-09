// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_GEOMETRY_BOUNDING_SHAPE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_GEOMETRY_BOUNDING_SHAPE_H_

#include "spacemit_mapping/core/types.h"

#include <variant>

namespace spacemit_mapping
{

struct BoundingSphere
{
    Vector3f center;
    float radius = 0;
};

struct AxisAlignedBox
{
    Vector3f minimum;
    Vector3f maximum;
};

class BoundingShape
{
public:
    explicit BoundingShape(BoundingSphere sphere);
    explicit BoundingShape(AxisAlignedBox box);
    bool contains(Vector3f point) const;
    // Inclusive contact with a block's closed bounds, including shared faces.
    bool touches_block(Index3D index, float block_size_m) const;

private:
    std::variant<BoundingSphere, AxisAlignedBox> shape_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_GEOMETRY_BOUNDING_SHAPE_H_
