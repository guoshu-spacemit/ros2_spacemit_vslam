/**
 * @file gles_render_pipeline.cpp
 * @brief Schedule bounded map updates and GPU rendering independently of fusion.
 */
#include "esdf_drm/gles_render_pipeline.h"
#include "feature_overlay_panel.h"
#include "performance_hud.h"
#include "esdf_drm/block_removal_queue.h"
#include "esdf_drm/trajectory_window.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <unistd.h>

namespace esdf_drm
{

GlesRenderPipeline::GlesRenderPipeline(GlesRenderPipelineConfig config, LogCallback error_callback,
    LogCallback info_callback, FrameCallback frame_callback)
    : config_(std::move(config)), error_callback_(std::move(error_callback)),
        info_callback_(std::move(info_callback)), frame_callback_(std::move(frame_callback))
{
    if (config_.target_fps <= 0.0 || config_.target_fps > 240.0 ||
        config_.renderer.maximum_blocks == 0 || config_.max_block_updates_per_frame == 0 ||
        config_.voxel_size_m <= 0.0F || config_.minimum_weight <= 0.0F ||
        config_.surface_band_m <= 0.0F || config_.slice_min_z_m > config_.slice_max_z_m ||
        !std::isfinite(config_.trajectory_duration_s) || config_.trajectory_duration_s < 0.0 ||
        config_.trajectory_duration_s > 3600.0)
    {
        throw std::invalid_argument("invalid GLES render pipeline configuration");
    }
}

GlesRenderPipeline::~GlesRenderPipeline() { stop(); }

void GlesRenderPipeline::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_)
    {
        return;
    }
    stopping_.store(false);
    started_ = true;
    thread_ = std::thread(&GlesRenderPipeline::render_loop, this);
}

void GlesRenderPipeline::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_)
        {
            return;
        }
        stopping_.store(true);
    }
    condition_.notify_all();
    if (thread_.joinable())
    {
        thread_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
    pending_tsdf_updates_.clear();
    pending_secondary_updates_.clear();
    if (pending_nv12_overlay_.dma_fd >= 0)
    {
        close(pending_nv12_overlay_.dma_fd);
        pending_nv12_overlay_ = {};
    }
    if (pending_feature_overlay_.dma_fd >= 0)
    {
        close(pending_feature_overlay_.dma_fd);
        pending_feature_overlay_ = {};
    }
    stats_.pending_blocks = 0;
}

void GlesRenderPipeline::submitDualView(const std::array<float, 16>& primary_view_projection,
    const std::array<float, 16>& secondary_view_projection,
    const esdf_drm::mapping::Transform& world_from_camera, std::int64_t source_timestamp_ns)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stopping_.load())
    {
        return;
    }
    view_projection_ = primary_view_projection;
    secondary_view_projection_ = secondary_view_projection;
    view_projection_ready_ = true;
    world_from_camera_ = world_from_camera;
    camera_timestamp_ns_ = source_timestamp_ns;
    camera_pose_ready_ = true;
    condition_.notify_one();
}

void GlesRenderPipeline::submitSecondaryUpdates(
    std::vector<esdf_drm::mapping::GlesBlockUpdate> updates)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!started_ || stopping_.load())
    {
        return;
    }
    ++stats_.submitted_batches;
    stats_.submitted_blocks += updates.size();
    for (auto& update : updates)
    {
        if (update.removed)
            wait_for_removal_slot(pending_secondary_updates_, update.index,
                config_.renderer.maximum_blocks, lock, condition_, stopping_);
        if (stopping_.load())
            return;
        auto existing = pending_secondary_updates_.find(update.index);
        if (existing != pending_secondary_updates_.end())
        {
            existing->second = std::move(update);
            ++stats_.coalesced_blocks;
        }
        else if (pending_secondary_updates_.size() < config_.renderer.maximum_blocks)
        {
            pending_secondary_updates_.emplace(update.index, std::move(update));
        }
        else
        {
            ++stats_.dropped_blocks;
        }
    }
    stats_.pending_blocks = pending_tsdf_updates_.size() + pending_secondary_updates_.size();
    condition_.notify_one();
}

void GlesRenderPipeline::submitTsdfUpdates(std::vector<esdf_drm::mapping::TsdfBlockUpdate> updates)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!started_ || stopping_.load())
    {
        return;
    }
    queue_tsdf_updates_locked(std::move(updates), lock);
    condition_.notify_one();
}

void GlesRenderPipeline::submitNv12Overlay(esdf_drm::mapping::GlesNv12OverlayFrame frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stopping_.load())
    {
        if (frame.dma_fd >= 0)
        {
            close(frame.dma_fd);
        }
        return;
    }
    if (pending_nv12_overlay_.dma_fd >= 0)
    {
        close(pending_nv12_overlay_.dma_fd);
    }
    pending_nv12_overlay_ = frame;
    condition_.notify_one();
}

void GlesRenderPipeline::submitFeatureOverlay(FeatureOverlayFrame frame)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!config_.feature_overlay_enabled || !started_ || stopping_.load())
    {
        if (frame.dma_fd >= 0)
            close(frame.dma_fd);
        return;
    }
    if (pending_feature_overlay_.dma_fd >= 0)
        close(pending_feature_overlay_.dma_fd);
    pending_feature_overlay_ = std::move(frame);
    condition_.notify_one();
}

void GlesRenderPipeline::queue_tsdf_updates_locked(
    std::vector<esdf_drm::mapping::TsdfBlockUpdate> updates, std::unique_lock<std::mutex>& lock)
{
    ++stats_.submitted_batches;
    stats_.submitted_blocks += updates.size();
    for (auto& update : updates)
    {
        if (update.removed)
            wait_for_removal_slot(pending_tsdf_updates_, update.index,
                config_.renderer.maximum_blocks, lock, condition_, stopping_);
        if (stopping_.load())
            return;
        auto existing = pending_tsdf_updates_.find(update.index);
        if (existing != pending_tsdf_updates_.end())
        {
            existing->second = std::move(update);
            ++stats_.coalesced_blocks;
            continue;
        }
        if (pending_tsdf_updates_.size() >= config_.renderer.maximum_blocks)
        {
            ++stats_.dropped_blocks;
            continue;
        }
        pending_tsdf_updates_.emplace(update.index, std::move(update));
    }
    stats_.pending_blocks = pending_tsdf_updates_.size() + pending_secondary_updates_.size();
}

GlesRenderPipelineStats GlesRenderPipeline::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void GlesRenderPipeline::render_loop()
{
    const NotifyStopped notify_stopped{stopping_, condition_, mutex_};
    esdf_drm::mapping::GlesDrmRenderer renderer(config_.renderer);
    std::string error;
    if (!renderer.initialize(&error))
    {
        if (error_callback_)
        {
            error_callback_("GLES/DRM initialization failed: " + error);
        }
        return;
    }
    renderer.setRenderMode(esdf_drm::mapping::GlesRenderMode::kTsdfPointsEsdfSlice);
    PerformanceHud hud;
    const bool hud_ready = config_.performance && hud.initialize(error);
    if (config_.performance && !hud_ready && error_callback_)
        error_callback_("GLES performance HUD failed: " + error);
    FeatureOverlayPanel feature_panel;
    const bool feature_panel_ready = config_.feature_overlay_enabled &&
        feature_panel.initialize(error, config_.feature_overlay_age_max_frames);
    if (config_.feature_overlay_enabled && !feature_panel_ready && error_callback_)
        error_callback_("GLES feature overlay failed: " + error);
    if (info_callback_)
    {
        info_callback_("GLES/DRM renderer ready: " + std::to_string(renderer.outputWidth()) + "x" +
            std::to_string(renderer.outputHeight()));
    }

    const auto frame_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / config_.target_fps));
    auto next_frame = std::chrono::steady_clock::now();
    auto next_report = next_frame + std::chrono::seconds(2);
    std::unique_ptr<TrajectoryWindow> trajectory;
    if (config_.trajectory_duration_s > 0.0)
        trajectory = std::make_unique<TrajectoryWindow>(config_.trajectory_duration_s,
            config_.renderer.maximum_trajectory_points, config_.renderer.trajectory_step_m);
    while (!stopping_.load())
    {
        std::vector<esdf_drm::mapping::GlesBlockUpdate> secondary_updates;
        std::vector<esdf_drm::mapping::TsdfBlockUpdate> tsdf_updates;
        std::array<float, 16> view_projection{};
        std::array<float, 16> secondary_view_projection{};
        bool view_ready = false;
        esdf_drm::mapping::Transform world_from_camera;
        bool camera_pose_ready = false;
        std::int64_t camera_timestamp_ns = 0;
        esdf_drm::mapping::GlesNv12OverlayFrame nv12_overlay;
        FeatureOverlayFrame feature_overlay;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_until(lock, next_frame, [this] { return stopping_.load(); });
            if (stopping_.load())
            {
                break;
            }
            tsdf_updates =
                take_render_updates(pending_tsdf_updates_, config_.max_block_updates_per_frame);
            secondary_updates = take_render_updates(
                pending_secondary_updates_, config_.max_block_updates_per_frame);
            stats_.pending_blocks =
                pending_tsdf_updates_.size() + pending_secondary_updates_.size();
            view_projection = view_projection_;
            secondary_view_projection = secondary_view_projection_;
            view_ready = view_projection_ready_;
            world_from_camera = world_from_camera_;
            camera_pose_ready = camera_pose_ready_;
            camera_timestamp_ns = camera_timestamp_ns_;
            nv12_overlay = pending_nv12_overlay_;
            pending_nv12_overlay_ = {};
            feature_overlay = std::move(pending_feature_overlay_);
            pending_feature_overlay_ = {};
        }
        condition_.notify_all();
        if (!tsdf_updates.empty() &&
            !renderer.updateTsdfBlocksCompute(tsdf_updates, config_.voxel_size_m,
                config_.minimum_weight, config_.surface_band_m, config_.slice_min_z_m,
                config_.slice_max_z_m, &error))
        {
            if (error_callback_)
            {
                error_callback_("GLES TSDF compute update failed: " + error);
            }
        }
        if (!secondary_updates.empty() &&
            !renderer.updateSecondaryBlocks(secondary_updates, &error))
        {
            if (error_callback_)
            {
                error_callback_("GLES secondary block update failed: " + error);
            }
        }
        if (view_ready)
        {
            renderer.setViewProjection(view_projection);
            renderer.setSecondaryViewProjection(secondary_view_projection);
        }
        if (camera_pose_ready && !renderer.updateCameraPose(world_from_camera, &error))
        {
            if (error_callback_)
            {
                error_callback_("GLES camera overlay failed: " + error);
            }
        }
        if (camera_pose_ready && trajectory &&
            trajectory->update(camera_timestamp_ns, world_from_camera.t))
        {
            const auto points = trajectory->positions();
            if (!renderer.updateCameraTrajectory(points, &error) && error_callback_)
                error_callback_("GLES trajectory window failed: " + error);
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.trajectory_points = points.size();
            stats_.trajectory_span_s = trajectory->span_seconds();
        }
        if (nv12_overlay.dma_fd >= 0)
        {
            if (!renderer.updateNv12Overlay(nv12_overlay, &error))
            {
                if (error_callback_)
                {
                    error_callback_("GLES NV12 preview overlay failed: " + error);
                }
            }
            close(nv12_overlay.dma_fd);
        }
        if (feature_overlay.dma_fd >= 0)
        {
            if (feature_panel_ready && !feature_panel.update(feature_overlay, error) &&
                error_callback_)
            {
                error_callback_("GLES feature overlay update failed: " + error);
            }
            close(feature_overlay.dma_fd);
            feature_overlay.source_lease.reset();
        }
        const auto draw_overlay = [&](int width, int height)
        {
            if (feature_panel_ready)
                feature_panel.draw(width, height, config_.preview_overlay_enabled);
            if (hud_ready)
                hud.draw(width, height, *config_.performance);
        };
        if (!renderer.renderFrame(&error, draw_overlay))
        {
            if (error_callback_)
            {
                error_callback_("GLES frame rendering failed: " + error);
            }
            break;
        }
        if (frame_callback_)
        {
            esdf_drm::mapping::GlesRenderedFrame frame;
            if (!renderer.currentFrame(&frame, &error))
            {
                if (error_callback_)
                {
                    error_callback_("GLES output frame unavailable: " + error);
                }
                break;
            }
            frame_callback_(frame);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.renderer = renderer.stats();
            stats_.rendered_frames = stats_.renderer.frames;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_report)
        {
            report_stats(renderer);
            next_report = now + std::chrono::seconds(2);
        }
        next_frame += frame_period;
        if (next_frame + frame_period < now)
        {
            next_frame = now;
        }
    }
    feature_panel.shutdown();
    hud.shutdown();
    renderer.shutdown();
}

void GlesRenderPipeline::report_stats(const esdf_drm::mapping::GlesDrmRenderer& renderer)
{
    if (!info_callback_)
    {
        return;
    }
    const auto renderer_stats = renderer.stats();
    const auto preparation = renderer.tsdf_prepare_stats();
    const std::uint64_t frame_delta = renderer_stats.frames - last_reported_frames_;
    const std::uint64_t block_delta = renderer_stats.updates - last_reported_updated_blocks_;
    const std::uint64_t byte_delta = renderer_stats.uploaded_bytes - last_reported_uploaded_bytes_;
    GlesRenderPipelineStats pipeline_stats;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pipeline_stats = stats_;
    }
    std::ostringstream message;
    message << std::fixed << std::setprecision(3) << "GLES map: frames=" << renderer_stats.frames
            << " blocks=" << renderer_stats.resident_blocks
            << " visible=" << renderer_stats.visible_blocks
            << " points=" << renderer_stats.resident_vertices << " gpu_geometry_mb="
            << (static_cast<double>(renderer_stats.gpu_capacity_bytes) / (1024.0 * 1024.0))
            << " multi_draw=" << (renderer_stats.indexed_multi_draw ? "yes" : "no")
            << " pending=" << pipeline_stats.pending_blocks
            << " coalesced=" << pipeline_stats.coalesced_blocks
            << " queue_dropped=" << pipeline_stats.dropped_blocks
            << " gpu_evicted=" << renderer_stats.evictions << " dirty_blocks/frame="
            << (frame_delta == 0U ? 0.0 : static_cast<double>(block_delta) / frame_delta)
            << " upload_bytes/frame="
            << (frame_delta == 0U ? 0.0 : static_cast<double>(byte_delta) / frame_delta)
            << " prepare=" << renderer_stats.prepare_ms << "ms"
            << " draw=" << renderer_stats.draw_ms << "ms"
            << " sync=" << renderer_stats.fence_ms << "ms"
            << " flip=" << renderer_stats.page_flip_ms << "ms"
            << " gpu_removed=" << renderer_stats.removals
            << " adapter[blocks=" << preparation.input_blocks << "/" << preparation.output_blocks
            << " apply_ms=" << preparation.apply_ms << " halo_ms=" << preparation.halo_ms
            << " submit_ms=" << preparation.submit_ms << "]"
            << " trail_points=" << pipeline_stats.trajectory_points
            << " trail_span_s=" << pipeline_stats.trajectory_span_s;
    if (config_.performance)
    {
        const auto now = EventWindow::Clock::now();
        const auto las2 = config_.performance->las2.sample(now);
        message << " HUD[mode=" << config_.performance->tracking_mode.sample(now)
                << " vio=" << config_.performance->vio.sample(now).fps
                << " tsdf=" << config_.performance->tsdf.sample(now).fps << " las2=" << las2.fps
                << " cl_gpu_ms=" << las2.mean_ms << "]";
    }
    info_callback_(message.str());
    last_reported_frames_ = renderer_stats.frames;
    last_reported_updated_blocks_ = renderer_stats.updates;
    last_reported_uploaded_bytes_ = renderer_stats.uploaded_bytes;
}

} // namespace esdf_drm
