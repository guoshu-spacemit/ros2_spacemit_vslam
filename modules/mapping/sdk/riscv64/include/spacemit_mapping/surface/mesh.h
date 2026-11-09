// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_SURFACE_MESH_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_SURFACE_MESH_H_

#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct MeshBlock
{
    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<Rgb8> colors;
    std::vector<std::int32_t> triangles;
    void clear() noexcept;
};

// Construct with voxel size (metres); blocks own their vectors and snapshots.
using MeshLayer = BlockLayer<MeshBlock>;

struct MeshParams
{
    float minimum_weight = 1.0e-4F;
    bool weld = true;
};

struct MeshUpdate
{
    // Resident input blocks in request order, including duplicates/empty output.
    std::vector<Index3D> processed_blocks;
};

class MeshExtractor
{
public:
    explicit MeshExtractor(MeshParams params = {});
    const MeshParams& params() const noexcept { return params_; }
    void set_params(MeshParams params);
    MeshUpdate update(
        const TsdfLayer& input, const std::vector<Index3D>& indices, MeshLayer& output) const;
    MeshUpdate update_all(const TsdfLayer& input, MeshLayer& output) const;

private:
    MeshParams params_;
};

// Stable millimetre-hash merge; retains the first normal and complete color.
// Throws before mutation for malformed vectors/indices or unhashable positions.
void weld_vertices(MeshBlock& mesh);

void update_mesh_colors(
    const ColorLayer& colors, const std::vector<Index3D>& indices, MeshLayer& output);
void update_mesh_colors(const ColorLayer& colors, MeshLayer& output);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_SURFACE_MESH_H_
