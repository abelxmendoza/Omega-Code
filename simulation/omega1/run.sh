#!/usr/bin/env bash
set -euo pipefail
omega_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command -v gz >/dev/null || { echo 'Install Gazebo Harmonic; gz was not found.' >&2; exit 1; }
python3 "$omega_dir/generate.py"
export GZ_SIM_RESOURCE_PATH="$omega_dir/generated/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -r "$omega_dir/generated/bench.sdf" "$@"
