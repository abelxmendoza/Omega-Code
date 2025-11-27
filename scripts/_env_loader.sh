# File: /Omega-Code/scripts/_env_loader.sh
# Summary:
# Safe environment loader for both backend and UI envs.
# Loads backend .env, then optional .env.local overrides. Use: `source ./_env_loader.sh`.

load_env() {
  local ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
  local ENV_BASE="${ENV_FILE:-$ROOT_DIR/servers/robot_controller_backend/.env}"
  local ENV_LOCAL="$ENV_BASE.local"

  set -o allexport
  [ -f "$ENV_BASE" ]  && . "$ENV_BASE"
  [ -f "$ENV_LOCAL" ] && . "$ENV_LOCAL"
  set +o allexport
}
