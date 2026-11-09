#!/usr/bin/env bash
# SDK custom-package entry; the SDK already owns the install lock here.
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 "$project_dir/tools/build.py" --sdk-entry "$@"
