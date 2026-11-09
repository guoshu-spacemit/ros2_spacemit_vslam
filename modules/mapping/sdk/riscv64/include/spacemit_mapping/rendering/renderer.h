// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_RENDERING_RENDERER_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_RENDERING_RENDERER_H_

#include "spacemit_mapping/core/image_view.h"
#include "spacemit_mapping/geometry/camera.h"
#include "spacemit_mapping/rendering/scene.h"

#include <functional>
#include <memory>
#include <string>

namespace spacemit_mapping::rendering
{

struct RendererOptions
{
    std::string render_device = "/dev/dri/renderD128";
    std::string display_device = "/dev/dri/card1";
    bool headless = true;
    int width = 1920, height = 1080;
    std::size_t maximum_blocks = 4096, maximum_vertices_per_block = 7680,
                maximum_indices_per_block = 7680;
    std::size_t maximum_trajectory_points = 8192;
    float point_size_pixels = 2, trajectory_step_m = 0.01F, camera_axis_m = 0.25F;
    std::array<float, 4> clear_color{0.015F, 0.020F, 0.030F, 1};
    bool frustum_culling = true, colored_cubes = false;
    bool wait_for_display = true;
    bool secondary_camera_indicator = true;
    float secondary_fov_degrees = 70, secondary_fov_range_m = 1.5F, secondary_marker_m = 0.12F;
    float secondary_marker_pixels = 18;
    float secondary_overlay_z_m = 0;
    int overlay_x = 16, overlay_y = 16, overlay_maximum_width = 0;
};
struct FrameView
{
    int dma_fd = -1, width = 0, height = 0, stride_bytes = 0;
    std::uint64_t number = 0;
};
struct Nv12View
{
    ImageView<std::uint8_t> y, uv; // UV width is measured in bytes, same as Y width.
};
struct Nv12Buffer
{
    int dma_fd = -1, width = 0, height = 0, stride_bytes = 0;
};
struct RawDepthView
{
    ImageView<float> depth;
    ImageView<Rgb8> rgb;
    PinholeCamera camera;
    float minimum_depth_m = 0.05F, maximum_depth_m = 10;
};
struct RenderStats
{
    std::uint64_t frames = 0, updates = 0, removals = 0, evictions = 0, uploaded_bytes = 0,
                    compute_dispatches = 0;
    std::size_t resident_blocks = 0, visible_blocks = 0, resident_vertices = 0,
                gpu_capacity_bytes = 0;
    bool indexed_multi_draw = false;
    double prepare_ms = 0, draw_ms = 0, fence_ms = 0, page_flip_ms = 0;
};

class Renderer
{
public:
    explicit Renderer(RendererOptions options = {});
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    void open();
    void close() noexcept;
    bool is_open() const noexcept;
    void set_view(ViewMode);
    void set_projection(const Matrix4&, SceneLayer = SceneLayer::kPrimary);
    void set_point_size(float pixels);
    void update(const std::vector<GeometryBlock>&, SceneLayer = SceneLayer::kPrimary);
    void clear(SceneLayer = SceneLayer::kPrimary);
    void set_camera(const RigidTransform&);
    void replace_trajectory(const std::vector<Vector3f>&);
    void set_raw_depth(const RawDepthView&);
    void set_overlay(Nv12View);
    void set_overlay(Nv12Buffer);
    void clear_images();
    FrameView render(const std::function<void(int, int)>& draw_overlay = {});
    FrameView current_frame() const;
    const RenderStats& stats() const noexcept;

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace spacemit_mapping::rendering

#endif // SPACEMIT_MAPPING_RENDERING_RENDERER_H_
