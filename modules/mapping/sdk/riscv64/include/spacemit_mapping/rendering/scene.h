// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_RENDERING_SCENE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_RENDERING_SCENE_H_

#include "spacemit_mapping/distance_field/field.h"
#include "spacemit_mapping/geometry/rigid_transform.h"
#include "spacemit_mapping/surface/mesh.h"

#include <limits>

namespace spacemit_mapping::rendering
{

using Matrix4 = std::array<float, 16>; // Column-major, clip_from_world.
inline constexpr Matrix4 kIdentityMatrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
enum class Primitive
{
    kPoints,
    kTriangles,
    kCubes,
    kSquares
};
enum class Generator
{
    kNone,
    kSurfacePoints,
    kMesh,
    kIndexedMesh,
    kSurfaceBand,
    kOccupiedVolume,
    kOccupiedSlice
};
enum class SceneLayer
{
    kPrimary,
    kSecondary
};
enum class ViewMode
{
    kSurfacePoints,
    kSurfaceMesh,
    kDistanceVolume,
    kDistanceSlice,
    kOccupiedVolume,
    kOccupiedSlice,
    kSurfaceAndDistance
};

struct Vertex
{
    Vector3f position;
    std::uint32_t rgba = 0xffffffffU;
};
struct GridSample
{
    float distance_m = 0, weight = 0;
    std::uint32_t rgba = 0xffffffffU, padding = 0;
};
inline constexpr std::size_t kHaloSamples = 9 * 9 * 9;
static_assert(
    sizeof(Vertex) == 16 && sizeof(GridSample) == 16, "GPU sample layout must be 16 bytes");

struct GeometryBlock
{
    Index3D index;
    float voxel_size_m = 0;
    bool removed = false;
    Primitive primitive = Primitive::kPoints;
    Generator generator = Generator::kNone;
    float minimum_weight = 1;
    float surface_band_m = -1;
    float minimum_z_m = -std::numeric_limits<float>::max();
    float maximum_z_m = std::numeric_limits<float>::max();
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<GridSample> grid; // Exactly kHaloSamples when generated on GPU.
};
struct SurfaceOptions
{
    float minimum_weight = 1;
    float surface_band_m = -1; // Negative selects one voxel.
};
struct GridOptions
{
    float minimum_weight = 1;
    float surface_band_m = -1;
    float minimum_z_m = -std::numeric_limits<float>::max();
    float maximum_z_m = std::numeric_limits<float>::max();
};
std::vector<GeometryBlock> surface_points(
    const TsdfLayer&, const std::vector<Index3D>&, SurfaceOptions = {});
std::vector<GeometryBlock> surface_mesh(
    const TsdfLayer&, const std::vector<Index3D>&, float minimum_weight = 1);
std::vector<GeometryBlock> tsdf_grid(
    const TsdfLayer&, const std::vector<Index3D>&, Generator, GridOptions = {});
std::vector<GeometryBlock> occupied_cells(const TsdfLayer&, const std::vector<Index3D>&,
    float minimum_weight, float minimum_z_m, float maximum_z_m, bool slice = false);
std::vector<GeometryBlock> distance_cells(
    const EsdfLayer&, const std::vector<Index3D>&, float maximum_distance_m, bool slice = false);
std::vector<GeometryBlock> mesh_geometry(const MeshLayer&, const std::vector<Index3D>&);
bool visible_block(Index3D, float voxel_size_m, const Matrix4&);
std::uint32_t pack_color(Rgb8 color, std::uint8_t alpha = 255) noexcept;

} // namespace spacemit_mapping::rendering

#endif // SPACEMIT_MAPPING_RENDERING_SCENE_H_
