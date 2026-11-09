# A visual observation in the source camera image.
uint64 id
float32 x
float32 y
# Selected as a same-frame 3D-backed PnP input in this camera (before rejection).
# This does not mean that the solver succeeded or accepted it as a final inlier.
bool pnp_input
# Continuous left-camera observation history across processed VIO frames.
# Source image time, NOT wall time or 3D landmark age/confidence. First = 0 s / 1.
# Missing observations or a tracker session reset restart this history.
float32 track_age_seconds
uint32 track_observation_count
