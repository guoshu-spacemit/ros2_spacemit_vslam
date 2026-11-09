// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_SURFACE_TOPOLOGY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_SURFACE_TOPOLOGY_H_

#include <cstdint>

namespace spacemit_mapping
{
// The same attributed topology used by CPU extraction and GPU geometry preparation.
// Sixteen entries, terminated by -1; storage lives for the program lifetime.
const std::int8_t* cube_triangle_edges(std::uint8_t configuration) noexcept;
} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_SURFACE_TOPOLOGY_H_
