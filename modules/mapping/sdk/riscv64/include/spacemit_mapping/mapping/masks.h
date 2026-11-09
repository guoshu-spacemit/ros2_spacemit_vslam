// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_MASKS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_MASKS_H_

#include "spacemit_mapping/geometry/camera.h"
#include "spacemit_mapping/geometry/rigid_transform.h"
#include "spacemit_mapping/core/image_view.h"

#include <vector>

namespace spacemit_mapping
{

struct ImageBox
{
    int left = 0, top = 0, right = -1, bottom = -1;
};

ImageBox clip_image_box(ImageBox box, int width, int height);
std::vector<std::uint8_t> clean_mask(ImageView<std::uint8_t> mask, int minimum_area);
std::vector<std::uint8_t> mask_from_boxes(
    ImageView<float> depth, const std::vector<ImageBox>& boxes, float mode_proximity_m = 0.2F);

struct ColorSplit
{
    std::size_t width = 0, height = 0;
    std::vector<Rgb8> background, foreground, overlay;
};

struct DepthSplit
{
    std::size_t width = 0, height = 0;
    std::vector<float> background, foreground;
    std::vector<Rgb8> overlay;
};

struct DepthSplitParams
{
    float occlusion_threshold_m = 0.25F;
    float background_invalid = -1;
    float foreground_invalid = -1;
};

ColorSplit split_color(ImageView<Rgb8> image, ImageView<std::uint8_t> mask);
DepthSplit split_depth(ImageView<float> depth, const PinholeCamera& depth_camera,
    ImageView<std::uint8_t> mask, const PinholeCamera& mask_camera,
    const RigidTransform& mask_from_depth, DepthSplitParams params = {});

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_MASKS_H_
