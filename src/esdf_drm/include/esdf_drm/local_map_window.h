#ifndef LOCAL_MAP_WINDOW_H
#define LOCAL_MAP_WINDOW_H
#include "esdf_drm/mapping_adapter.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace esdf_drm
{

// Single owner: the mapping worker. Radius zero preserves the unbounded map.
class LocalMapWindow
{
public:
    using Clock = std::chrono::steady_clock;

    LocalMapWindow(
        const esdf_drm::mapping::IntegratorParams& params, double radius_m, double rate_hz)
        : mapper_(params), radius_m_(radius_m)
    {
        if (!std::isfinite(radius_m) || radius_m < 0.0 ||
            radius_m > std::numeric_limits<float>::max() || !std::isfinite(rate_hz) ||
            rate_hz <= 0.0 || rate_hz > 60.0)
            throw std::invalid_argument(
                "map clearing requires finite float radius >= 0 and rate in (0,60]");
        period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / rate_hz));
    }

    esdf_drm::mapping::TsdfMapper& tsdf() { return mapper_; }

    std::vector<esdf_drm::mapping::Index3D> clear(
        const esdf_drm::mapping::Vector3f& center, Clock::time_point now = Clock::now())
    {
        if (radius_m_ == 0.0 || (started_ && now < next_clear_))
            return {};
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z))
            throw std::invalid_argument("non-finite map clearing center");
        auto removed = mapper_.clearOutsideRadius(center, static_cast<float>(radius_m_));
        started_ = true;
        next_clear_ = now + period_;
        return removed;
    }

private:
    esdf_drm::mapping::TsdfMapper mapper_;
    double radius_m_;
    Clock::duration period_{};
    Clock::time_point next_clear_{};
    bool started_{false};
};

// Mirror spacemit_mapping Mapper's dependent-layer removal rule at the asynchronous
// transport boundary: retain a slice column while any input block supports it.
// Call AFTER integrateSlice() has invalidated distances to removed sites.
inline std::vector<esdf_drm::mapping::Index3D> clear_removed_esdf_blocks(
    const esdf_drm::mapping::TsdfMapper& tsdf, esdf_drm::mapping::EsdfIntegrator& esdf,
    const std::vector<esdf_drm::mapping::Index3D>& removed, float min_z, float max_z,
    float output_z)
{
    const float block_size = esdf.voxelSize() * esdf_drm::mapping::kVoxelsPerSide;
    const int first_z = static_cast<int>(std::floor(min_z / block_size));
    const int last_z = static_cast<int>(std::floor(max_z / block_size));
    const int slice_z = static_cast<int>(std::floor(output_z / block_size));
    std::vector<esdf_drm::mapping::Index3D> cleared_columns;
    for (const auto& index : removed)
    {
        esdf.blocks().erase(index);
        bool supported = false;
        for (int z = first_z; z <= last_z && !supported; ++z)
            supported = tsdf.blocks().count({index.x, index.y, z}) != 0;
        const esdf_drm::mapping::Index3D column{index.x, index.y, slice_z};
        if (!supported && esdf.sliceBlocks().erase(column) != 0)
            cleared_columns.push_back(column);
    }
    return cleared_columns;
}

} // namespace esdf_drm

#endif // LOCAL_MAP_WINDOW_H
