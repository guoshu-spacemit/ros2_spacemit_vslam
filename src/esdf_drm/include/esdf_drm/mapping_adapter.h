#ifndef MAPPING_ADAPTER_H
#define MAPPING_ADAPTER_H
#include <spacemit_mapping/mapping/session.h>
#include <spacemit_mapping/rendering/renderer.h>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Application-owned boundary. Map storage, fusion, distance propagation and
// rendering are implemented exclusively by the independently packaged SDK.
namespace esdf_drm::mapping
{
using spacemit_mapping::Backend;
using spacemit_mapping::Index3D;
using spacemit_mapping::Index3DHash;
using spacemit_mapping::kVoxelsPerBlock;
using spacemit_mapping::kVoxelsPerSide;
using spacemit_mapping::TsdfBlock;
using spacemit_mapping::Vector3f;
using TsdfBlockUpdate = spacemit_mapping::BlockUpdate<TsdfBlock>;
using GlesBlockUpdate = spacemit_mapping::rendering::GeometryBlock;
using GlesRenderedFrame = spacemit_mapping::rendering::FrameView;
using GlesNv12OverlayFrame = spacemit_mapping::rendering::Nv12Buffer;
using GlesDrmRendererStats = spacemit_mapping::rendering::RenderStats;

struct Transform
{
    std::array<float, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<float, 3> t{0, 0, 0};
};
inline spacemit_mapping::RigidTransform native_pose(const Transform& pose)
{
    return {pose.R, {pose.t[0], pose.t[1], pose.t[2]}};
}
struct Camera
{
    int width = 0, height = 0;
    float fx = 0, fy = 0, cx = 0, cy = 0;
};
struct IntegratorParams
{
    float voxel_size_m = 0.05F;
    float truncation_distance_m = 0.2F;
    float max_integration_distance_m = 7;
    float invalid_depth_decay_factor = -1;
    Backend backend = Backend::kAuto;
};

class TsdfMapper
{
public:
    explicit TsdfMapper(const IntegratorParams& params = {}) : session_(params.voxel_size_m)
    {
        auto settings = session_.params();
        settings.tsdf.kernel.truncation_m = params.truncation_distance_m;
        settings.tsdf.kernel.invalid_depth_weight_factor = params.invalid_depth_decay_factor;
        settings.tsdf.maximum_distance_m = params.max_integration_distance_m;
        settings.tsdf.backend = params.backend;
        session_.set_params(settings);
    }
    spacemit_mapping::TsdfFrameStats integrateRgbd(const std::vector<float>& depth,
        const std::vector<spacemit_mapping::Rgb8>& rgb, const Camera& camera, const Transform& pose)
    {
        static_assert(sizeof(spacemit_mapping::Rgb8) == 3);
        const auto result = session_.integrate_camera(
            {depth.data(), depth.size(), static_cast<std::size_t>(camera.width),
                static_cast<std::size_t>(camera.height)},
            {camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy},
            native_pose(pose),
            rgb.empty() ? spacemit_mapping::ImageView<spacemit_mapping::Rgb8>{}
                        : spacemit_mapping::ImageView<spacemit_mapping::Rgb8>{rgb.data(),
                                rgb.size(), static_cast<std::size_t>(camera.width),
                                static_cast<std::size_t>(camera.height)});
        changed_ = result.updated_blocks;
        return result.tsdf;
    }
    auto& blocks() { return session_.tsdf().blocks(); }
    const auto& blocks() const { return session_.tsdf().blocks(); }
    std::size_t numBlocks() const { return session_.tsdf().size(); }
    const auto& lastUpdatedBlocks() const { return changed_; }
    auto copyBlockUpdates(const std::vector<Index3D>& indices) const
    {
        return session_.tsdf().copy_updates(indices);
    }
    void applyBlockUpdates(const std::vector<TsdfBlockUpdate>& updates)
    {
        changed_ = session_.tsdf().apply_updates(updates);
    }
    spacemit_mapping::TsdfLayer& layer() { return session_.tsdf(); }
    const spacemit_mapping::TsdfLayer& layer() const { return session_.tsdf(); }
    std::vector<Index3D> clearOutsideRadius(Vector3f center, float radius)
    {
        return session_.erase_outside_sphere({center, radius});
    }

private:
    spacemit_mapping::MapSession session_;
    std::vector<Index3D> changed_;
};

struct EsdfParams
{
    float max_distance_m = 2;
    Backend backend = Backend::kAuto;
};
using EsdfStats = spacemit_mapping::DistanceFieldStats;
class EsdfIntegrator
{
public:
    explicit EsdfIntegrator(float voxel_size, const EsdfParams& params = {})
        : field_(voxel_size, native_params(params))
    {
    }
    EsdfStats integrateBlocks(const TsdfMapper& source, const std::vector<Index3D>& indices)
    {
        return field_.update(source.layer(), indices).stats;
    }
    EsdfStats integrateSlice(const TsdfMapper& source, const std::vector<Index3D>& indices,
        float minimum_z, float maximum_z, float output_z)
    {
        auto result = field_.update_slice(
            source.layer(), indices, spacemit_mapping::HeightBand{minimum_z, maximum_z, output_z});
        changed_slice_ = std::move(result.updated_blocks);
        return result.stats;
    }
    EsdfStats integrateSliceLayer(
        const TsdfMapper& source, float minimum_z, float maximum_z, float output_z)
    {
        return integrateSlice(source, source.layer().indices(), minimum_z, maximum_z, output_z);
    }
    const auto& lastUpdatedSliceBlocks() const { return changed_slice_; }
    auto& blocks() { return field_.layer().blocks(); }
    auto& sliceBlocks() { return field_.slice_layer().blocks(); }
    std::size_t numSliceBlocks() const { return field_.slice_layer().size(); }
    float voxelSize() const { return field_.voxel_size(); }
    const spacemit_mapping::DistanceField& field() const { return field_; }

private:
    static spacemit_mapping::DistanceFieldParams native_params(const EsdfParams& params)
    {
        spacemit_mapping::DistanceFieldParams result;
        result.maximum_distance_m = params.max_distance_m;
        result.backend = params.backend;
        return result;
    }
    spacemit_mapping::DistanceField field_;
    std::vector<Index3D> changed_slice_;
};
inline auto makeEsdfBlockUpdates(
    const EsdfIntegrator& source, const std::vector<Index3D>& indices, bool slice)
{
    return spacemit_mapping::rendering::distance_cells(
        slice ? source.field().slice_layer() : source.field().layer(), indices,
        source.field().params().maximum_distance_m, slice);
}

struct GlesDrmRendererConfig : spacemit_mapping::rendering::RendererOptions
{
    bool interpolate_tsdf_surface_points = true;
    bool split_view = true;
};
enum class GlesRenderMode
{
    kTsdfPointsEsdfSlice
};

// Preserve the application's bounded producer/consumer contracts. No ownership
// of a borrowed DMA-BUF is extended beyond the native synchronous upload call.
class GlesDrmRenderer
{
public:
    struct TsdfPrepareStats
    {
        double apply_ms = 0, halo_ms = 0, submit_ms = 0;
        std::size_t input_blocks = 0, output_blocks = 0;
    };
    explicit GlesDrmRenderer(GlesDrmRendererConfig config) : config_(std::move(config)) {}
    bool initialize(std::string* error)
    {
        return checked(error,
            [&]
            {
                renderer_ = std::make_unique<spacemit_mapping::rendering::Renderer>(config_);
                renderer_->open();
                renderer_->render();
            });
    }
    void shutdown()
    {
        if (renderer_)
            renderer_->close();
    }
    void setRenderMode(GlesRenderMode)
    {
        renderer_->set_view(config_.split_view
                ? spacemit_mapping::rendering::ViewMode::kSurfaceAndDistance
                : spacemit_mapping::rendering::ViewMode::kSurfacePoints);
    }
    int outputWidth() const { return renderer_->current_frame().width; }
    int outputHeight() const { return renderer_->current_frame().height; }
    void setViewProjection(const std::array<float, 16>& projection)
    {
        renderer_->set_projection(projection);
    }
    void setSecondaryViewProjection(const std::array<float, 16>& projection)
    {
        renderer_->set_projection(projection, spacemit_mapping::rendering::SceneLayer::kSecondary);
    }
    bool updateCameraPose(const Transform& pose, std::string* error)
    {
        return checked(error, [&] { renderer_->set_camera(native_pose(pose)); });
    }
    bool updateCameraTrajectory(const std::vector<std::array<float, 3>>& points, std::string* error)
    {
        return checked(error,
            [&]
            {
                std::vector<Vector3f> converted;
                converted.reserve(points.size());
                for (const auto& p : points)
                    converted.push_back({p[0], p[1], p[2]});
                renderer_->replace_trajectory(converted);
            });
    }
    bool updateTsdfBlocksCompute(const std::vector<TsdfBlockUpdate>& updates, float voxel_size,
        float weight, float band, float minimum_z, float maximum_z, std::string* error)
    {
        return checked(error,
            [&]
            {
                const auto begin = std::chrono::steady_clock::now();
                if (!mirror_)
                    mirror_ = std::make_unique<spacemit_mapping::TsdfLayer>(voxel_size);
                if (mirror_->voxel_size() != voxel_size)
                    throw std::invalid_argument("renderer voxel size changed within a session");
                const auto changed = mirror_->apply_updates(updates);
                const auto applied = std::chrono::steady_clock::now();
                spacemit_mapping::rendering::GridOptions options;
                options.minimum_weight = weight;
                options.surface_band_m = band;
                options.minimum_z_m = minimum_z;
                options.maximum_z_m = maximum_z;
                const auto generator = config_.interpolate_tsdf_surface_points
                    ? spacemit_mapping::rendering::Generator::kSurfacePoints
                    : spacemit_mapping::rendering::Generator::kSurfaceBand;
                auto geometry =
                    spacemit_mapping::rendering::tsdf_grid(*mirror_, changed, generator, options);
                const auto prepared = std::chrono::steady_clock::now();
                renderer_->update(geometry);
                const auto finished = std::chrono::steady_clock::now();
                tsdf_prepare_ = {std::chrono::duration<double, std::milli>(applied - begin).count(),
                    std::chrono::duration<double, std::milli>(prepared - applied).count(),
                    std::chrono::duration<double, std::milli>(finished - prepared).count(),
                    updates.size(), geometry.size()};
            });
    }
    bool updateSecondaryBlocks(const std::vector<GlesBlockUpdate>& updates, std::string* error)
    {
        return checked(error, [&]
            { renderer_->update(updates, spacemit_mapping::rendering::SceneLayer::kSecondary); });
    }
    bool updateNv12Overlay(const GlesNv12OverlayFrame& frame, std::string* error)
    {
        return checked(error, [&] { renderer_->set_overlay(frame); });
    }
    bool renderFrame(std::string* error, const std::function<void(int, int)>& overlay = {})
    {
        return checked(error, [&] { renderer_->render(overlay); });
    }
    bool currentFrame(GlesRenderedFrame* frame, std::string* error) const
    {
        return checked(error, [&] { *frame = renderer_->current_frame(); });
    }
    const GlesDrmRendererStats& stats() const { return renderer_->stats(); }
    const TsdfPrepareStats& tsdf_prepare_stats() const { return tsdf_prepare_; }

private:
    template <class Function> static bool checked(std::string* error, Function&& operation)
    {
        try
        {
            operation();
            return true;
        }
        catch (const std::exception& exception)
        {
            if (error)
                *error = exception.what();
            return false;
        }
    }
    GlesDrmRendererConfig config_;
    std::unique_ptr<spacemit_mapping::rendering::Renderer> renderer_;
    std::unique_ptr<spacemit_mapping::TsdfLayer> mirror_;
    TsdfPrepareStats tsdf_prepare_;
};
} // namespace esdf_drm::mapping

#endif // MAPPING_ADAPTER_H
