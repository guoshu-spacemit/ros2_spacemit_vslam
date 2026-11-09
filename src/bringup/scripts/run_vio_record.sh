#!/usr/bin/env bash
# Same runtime overrides as run_headless_esdf_rtsp.sh; dedicated recording profile.
set -euo pipefail
script_dir=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
export SPACEMIT_ROBOT_RUN_DIR=${SPACEMIT_ROBOT_RUN_DIR:-/tmp/vio_record}
export SPACEMIT_ROBOT_ENABLE_SPACEMIT_MAPPING=false SPACEMIT_ROBOT_ENABLE_RTSP=false SPACEMIT_ROBOT_ENABLE_ESDF_RTSP=false
export SPACEMIT_ROBOT_PUBLISH_LAS2_PREVIEW=false
export SPACEMIT_ROBOT_RECORD_SESSION_DIR=${SPACEMIT_ROBOT_RECORD_SESSION_DIR:-${SPACEMIT_ROBOT_RECORD_OUTPUT_DIR:-${SPACEMIT_ROBOT_WORKSPACE_ROOT:?Set SPACEMIT_ROBOT_WORKSPACE_ROOT}/recordings}/recording_$(date +%Y%m%d_%H%M%S_%N)}
exec "${script_dir}/run_headless_esdf_rtsp.sh" "$@"
