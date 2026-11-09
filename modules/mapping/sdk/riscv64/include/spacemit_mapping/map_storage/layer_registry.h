// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAP_STORAGE_LAYER_REGISTRY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAP_STORAGE_LAYER_REGISTRY_H_

#include "spacemit_mapping/map_storage/layer.h"

#include <memory>
#include <stdexcept>
#include <typeindex>
#include <type_traits>

namespace spacemit_mapping
{

// A typed owner of shared layers; sharing a layer does not copy its blocks.
class LayerRegistry
{
public:
    explicit LayerRegistry(float voxel_size_m) : resolution_(voxel_size_m)
    {
        // Reuse the same size validation without allocating a resident block.
        TsdfLayer validated(voxel_size_m);
    }
    LayerRegistry(const LayerRegistry&) = delete;
    LayerRegistry& operator=(const LayerRegistry&) = delete;
    LayerRegistry(LayerRegistry&&) = default;
    LayerRegistry& operator=(LayerRegistry&&) = default;
    float voxel_size() const noexcept { return resolution_; }
    float block_size() const noexcept { return resolution_ * kVoxelsPerSide; }
    bool empty() const noexcept { return layers_.empty(); }
    std::size_t size() const noexcept { return layers_.size(); }
    void clear() noexcept { layers_.clear(); }

    template <class Layer> Layer* add()
    {
        static_assert(std::is_base_of_v<LayerBase, Layer>);
        if (get<Layer>())
            return nullptr;
        auto layer = std::make_shared<Layer>(resolution_);
        auto* result = layer.get();
        layers_.emplace(std::type_index(typeid(Layer)), std::move(layer));
        return result;
    }

    template <class Layer> bool insert(const std::shared_ptr<Layer>& layer)
    {
        static_assert(std::is_base_of_v<LayerBase, Layer>);
        if (!layer || layer->voxel_size() != resolution_)
            throw std::invalid_argument("Inserted layer must exist and have matching resolution");
        return layers_.emplace(std::type_index(typeid(Layer)), layer).second;
    }

    template <class Layer> Layer* get()
    {
        const auto found = layers_.find(std::type_index(typeid(Layer)));
        return found == layers_.end() ? nullptr : dynamic_cast<Layer*>(found->second.get());
    }

    template <class Layer> const Layer* get() const
    {
        const auto found = layers_.find(std::type_index(typeid(Layer)));
        return found == layers_.end() ? nullptr : dynamic_cast<const Layer*>(found->second.get());
    }

    template <class Layer> std::shared_ptr<Layer> shared()
    {
        const auto found = layers_.find(std::type_index(typeid(Layer)));
        return found == layers_.end() ? nullptr : std::dynamic_pointer_cast<Layer>(found->second);
    }

    template <class Layer> std::shared_ptr<const Layer> shared() const
    {
        const auto found = layers_.find(std::type_index(typeid(Layer)));
        return found == layers_.end() ? nullptr
                                        : std::dynamic_pointer_cast<const Layer>(found->second);
    }

private:
    float resolution_;
    std::unordered_map<std::type_index, std::shared_ptr<LayerBase>> layers_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAP_STORAGE_LAYER_REGISTRY_H_
