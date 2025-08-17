# File: /Omega-Code/scripts/net_profile.sh
# Summary:
# Helper to resolve profile-specific NEXT_PUBLIC_* env vars in Bash scripts.
# Usage: `URL=$(resolve_by_profile NEXT_PUBLIC_VIDEO_STREAM_URL)`

resolve_by_profile() {
  local base="$1"
  local profile="${NEXT_PUBLIC_NETWORK_PROFILE:-local}"
  local key="${base}_$(echo "$profile" | tr '[:lower:]' '[:upper:]')"
  # best-effort: convert any dashes to underscores
  key="${key//-/_}"
  # shellcheck disable=SC2086,SC2248
  eval "printf '%s' \"\${$key:-}\""
}

