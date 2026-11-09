// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAP_STORAGE_CHANGE_TRACKER_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAP_STORAGE_CHANGE_TRACKER_H_

#include "spacemit_mapping/core/types.h"

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace spacemit_mapping
{

using ConsumerId = std::uint32_t;
class ChangeTracker;

// Owning snapshot and acknowledgement ticket for one tracker/consumer epoch.
class UpdateBatch
{
public:
    bool full_update() const noexcept { return full_; }
    ConsumerId consumer() const noexcept { return consumer_; }
    std::uint64_t revision() const noexcept { return revision_; }
    const std::vector<Index3D>& blocks() const;

private:
    friend class ChangeTracker;
    UpdateBatch(std::shared_ptr<const char> owner, ConsumerId consumer, std::uint64_t generation,
        std::uint64_t revision, bool full, std::vector<Index3D> blocks);
    std::shared_ptr<const char> owner_;
    ConsumerId consumer_;
    std::uint64_t generation_, revision_;
    bool full_;
    std::vector<Index3D> blocks_;
};

class ChangeTracker
{
public:
    explicit ChangeTracker(std::size_t maximum_pending_blocks = 100000);
    ChangeTracker(const ChangeTracker&) = delete;
    ChangeTracker& operator=(const ChangeTracker&) = delete;
    // Empty consumer list broadcasts to all initialized consumers.
    void mark_changed(
        const std::vector<Index3D>& blocks, const std::vector<ConsumerId>& consumers = {});
    void mark_all();
    // Empty list broadcasts; nonempty list preserves all other consumer states.
    void mark_all(const std::vector<ConsumerId>& consumers);
    // Cancellation only. To deliver removals, mark_changed and copy tombstones.
    void forget(const std::vector<Index3D>& blocks);
    UpdateBatch snapshot(ConsumerId consumer);
    // Clear only changes at or before the batch revision; retain later writes.
    void acknowledge(const UpdateBatch& batch);
    void reset();

private:
    struct State
    {
        bool full = true;
        std::uint64_t full_revision = 0;
        std::unordered_map<Index3D, std::uint64_t, Index3DHash> pending;
    };
    std::uint64_t advance();
    void insert(State& state, const std::vector<Index3D>& blocks, std::uint64_t revision);
    std::mutex mutex_;
    std::shared_ptr<const char> identity_;
    std::size_t maximum_pending_;
    std::uint64_t revision_ = 0;
    std::uint64_t generation_ = 0;
    std::map<ConsumerId, State> states_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAP_STORAGE_CHANGE_TRACKER_H_
