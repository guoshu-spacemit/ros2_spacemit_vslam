// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_FREESPACE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_FREESPACE_H_

#include "spacemit_mapping/mapping/views.h"
#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct FreespaceParams
{
    float occupancy_distance_m = 0.15F;
    TimestampMs maximum_unobserved_interval_ms = 200;
    TimestampMs minimum_free_interval_ms = 1000;
    TimestampMs minimum_occupied_reset_ms = 2000;
    bool check_neighborhood = true;
    bool initialize_free = false;
};

using VoxelUpdateMask = std::array<std::uint8_t, kVoxelsPerBlock>;

// The output block is written only after a complete successful state step.
void update_freespace_block(const TsdfBlock& input, TimestampMs previous_update_ms,
    TimestampMs current_update_ms, const FreespaceParams& params, FreespaceBlock& output,
    const VoxelUpdateMask* mask = nullptr);

class FreespaceUpdater
{
public:
    explicit FreespaceUpdater(FreespaceParams params = {});
    const FreespaceParams& params() const noexcept { return params_; }
    void set_params(FreespaceParams params);
    TimestampMs last_update_time() const noexcept { return last_update_ms_; }
    void reset_clock() noexcept { last_update_ms_ = 0; }
    void restore_clock(TimestampMs time_ms);
    std::vector<Index3D> update(const TsdfLayer& input, const std::vector<Index3D>& indices,
        TimestampMs time_ms, FreespaceLayer& output);
    std::vector<Index3D> update(const TsdfLayer& input, const std::vector<Index3D>& indices,
        TimestampMs time_ms, FreespaceLayer& output, const CameraMapView& view);
    std::vector<Index3D> update(const TsdfLayer& input, const std::vector<Index3D>& indices,
        TimestampMs time_ms, FreespaceLayer& output, const LidarMapView& view);

private:
    std::vector<Index3D> apply(const TsdfLayer&, const std::vector<Index3D>&, TimestampMs,
        FreespaceLayer&, const CameraMapView*, const LidarMapView*);
    FreespaceParams params_;
    TimestampMs last_update_ms_ = 0;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_FREESPACE_H_
