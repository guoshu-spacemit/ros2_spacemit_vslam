// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAP_STORAGE_LAYER_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAP_STORAGE_LAYER_H_

#include "spacemit_mapping/map_storage/blocks.h"

#include <unordered_map>
#include <utility>
#include <vector>

namespace spacemit_mapping
{

class LayerBase
{
public:
    explicit LayerBase(float voxel_size_m);
    virtual ~LayerBase() = default;
    LayerBase(const LayerBase&) = delete;
    LayerBase& operator=(const LayerBase&) = delete;
    LayerBase(LayerBase&&) = default;
    LayerBase& operator=(LayerBase&&) = default;
    float voxel_size() const noexcept { return voxel_size_; }
    float block_size() const noexcept { return voxel_size_ * kVoxelsPerSide; }
    virtual std::size_t size() const noexcept = 0;
    virtual void clear() noexcept = 0;

private:
    float voxel_size_;
};

// Each value owns the full block, including for a removal (payload is ignored).
template <class Block> struct BlockUpdate
{
    Index3D index;
    Block block{};
    bool removed = false;
};

template <class Block> class BlockLayer final : public LayerBase
{
public:
    using BlockType = Block;
    using BlockMap = std::unordered_map<Index3D, Block, Index3DHash>;
    explicit BlockLayer(float voxel_size_m) : LayerBase(voxel_size_m) {}
    Block& allocate(Index3D index) { return blocks_.try_emplace(index).first->second; }
    Block* find(Index3D index)
    {
        const auto entry = blocks_.find(index);
        return entry == blocks_.end() ? nullptr : &entry->second;
    }
    const Block* find(Index3D index) const
    {
        const auto entry = blocks_.find(index);
        return entry == blocks_.end() ? nullptr : &entry->second;
    }
    bool contains(Index3D index) const { return find(index) != nullptr; }
    bool erase(Index3D index) { return blocks_.erase(index) != 0; }
    std::size_t size() const noexcept override { return blocks_.size(); }
    void clear() noexcept override { blocks_.clear(); }
    const BlockMap& blocks() const noexcept { return blocks_; }
    BlockMap& blocks() noexcept { return blocks_; }
    // Container traversal order; callers sort explicitly for canonical output.
    std::vector<Index3D> indices() const
    {
        std::vector<Index3D> result;
        result.reserve(size());
        for (const auto& entry : blocks_)
            result.push_back(entry.first);
        return result;
    }
    // Preserve requested order and duplicates. Missing blocks produce tombstones.
    std::vector<BlockUpdate<Block>> copy_updates(const std::vector<Index3D>& indices) const
    {
        std::vector<BlockUpdate<Block>> result;
        result.reserve(indices.size());
        for (Index3D index : indices)
        {
            BlockUpdate<Block> item;
            item.index = index;
            const Block* block = find(index);
            if (block)
                item.block = *block;
            else
                item.removed = true;
            result.push_back(std::move(item));
        }
        return result;
    }
    // Sequential last-write-wins; returns all input indices including removals.
    // Allocation failure can leave an applied prefix, matching live-map updates.
    std::vector<Index3D> apply_updates(const std::vector<BlockUpdate<Block>>& updates)
    {
        std::vector<Index3D> applied;
        applied.reserve(updates.size());
        for (const auto& item : updates)
        {
            if (item.removed)
                erase(item.index);
            else
                blocks_.insert_or_assign(item.index, item.block);
            applied.push_back(item.index);
        }
        return applied;
    }

private:
    BlockMap blocks_;
};

using TsdfLayer = BlockLayer<TsdfBlock>;
using ColorLayer = BlockLayer<ColorBlock>;
using OccupancyLayer = BlockLayer<OccupancyBlock>;
using EsdfLayer = BlockLayer<EsdfBlock>;
using FreespaceLayer = BlockLayer<FreespaceBlock>;

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAP_STORAGE_LAYER_H_
