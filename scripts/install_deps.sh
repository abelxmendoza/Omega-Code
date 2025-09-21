# File: /Omega-Code/scripts/install_deps.sh
# Summary:
#   Installs system-level deps for the scripts on macOS or Raspberry Pi (Debian).
#   No Python requirements needed for scripts unless you add Python helpers later.

set -euxo pipefail

OS="$(uname -s)"

if [[ "$OS" == "Darwin" ]]; then
  # --- macOS ---
  command -v brew >/dev/null || { echo "Homebrew required: https://brew.sh"; exit 1; }
  brew update
  brew install blueutil curl
  # Optional dev tools:
  command -v npm >/dev/null && npm i -g wscat@8 || true
  echo "✅ macOS deps installed."

elif [[ "$OS" == "Linux" ]]; then
  # --- Raspberry Pi / Debian ---
  export DEBIAN_FRONTEND=noninteractive
  sudo apt update
  sudo apt install -y \
    bluez bluez-tools rfkill \
    curl netcat-openbsd dnsutils iproute2 \
    dhcpcd5 dhclient

  # Optional camera packages (install if you use the Pi camera server)
  # sudo apt install -y python3-picamera2 python3-opencv

  echo "✅ Linux deps installed."
else
  echo "Unsupported OS: $OS"
  exit 1
fi
