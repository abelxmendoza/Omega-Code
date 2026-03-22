#!/usr/bin/env bash
# =============================================================================
# setup_pi_camera.sh -- Omega-1 Pi camera stack bootstrap
# =============================================================================
# Replays every system-level change made during the 2025-03 Pi camera bring-up
# so a fresh image or second Pi can be made camera-ready without manual steps.
#
# What this script does
# ---------------------
#   1. Installs system apt packages (python3-opencv with GStreamer support)
#   2. Removes pip opencv-python (no GStreamer), pins numpy<2 (cv_bridge compat)
#   3. Blacklists bcm2835_v4l2 kernel module (it grabs camera at boot and
#      conflicts with libcamera)
#   4. Installs pre-built libcamera 0.7.0 artifacts from the repo's bundled
#      copy at servers/robot_controller_backend/libcamera/  into system paths
#   5. Writes /etc/ld.so.conf.d/libcamera-local.conf and runs ldconfig
#   6. Adds required env vars to ~/.bashrc (idempotent)
#
# Prerequisites
# -------------
#   - Raspberry Pi running Ubuntu 22.04 / Debian Bullseye (aarch64)
#   - libcamera 0.7.0 build artifacts already in the repo under
#     servers/robot_controller_backend/libcamera/
#     (built from source as documented in docs/pi_camera_build.md)
#   - Run as the robot user (omega1), NOT as root -- script uses sudo internally
#
# Usage
# -----
#   chmod +x scripts/setup_pi_camera.sh
#   ./scripts/setup_pi_camera.sh
#
# Safe to re-run (idempotent where possible).
# =============================================================================

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBCAM_SRC="${REPO_ROOT}/servers/robot_controller_backend/libcamera"

# Colours
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[setup]${NC} $*"; }
warn()    { echo -e "${YELLOW}[warn]${NC}  $*"; }
error()   { echo -e "${RED}[error]${NC} $*" >&2; }

# ---------------------------------------------------------------------------
# 0. Sanity checks
# ---------------------------------------------------------------------------
if [[ "$(uname -m)" != "aarch64" ]]; then
    warn "Not running on aarch64 -- some steps may not apply (detected: $(uname -m))"
fi

if [[ $EUID -eq 0 ]]; then
    error "Run this script as the robot user (omega1), not root."
    error "sudo is invoked internally where needed."
    exit 1
fi

# ---------------------------------------------------------------------------
# 1. System apt packages
# ---------------------------------------------------------------------------
info "Step 1/6: Installing system apt packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

# ---------------------------------------------------------------------------
# 2. pip: remove opencv-python, pin numpy<2
# ---------------------------------------------------------------------------
info "Step 2/6: Adjusting pip packages (opencv-python -> system, numpy<2)..."

# Remove pip opencv-python -- it was compiled without GStreamer support
if pip3 show opencv-python &>/dev/null 2>&1; then
    pip3 uninstall -y opencv-python
    info "  Removed pip opencv-python."
else
    info "  pip opencv-python not installed -- skipping."
fi

if pip3 show opencv-contrib-python &>/dev/null 2>&1; then
    pip3 uninstall -y opencv-contrib-python
    info "  Removed pip opencv-contrib-python."
fi

# Pin numpy below 2 -- system apt OpenCV (4.5.4) and cv_bridge were compiled
# against NumPy 1.x; NumPy 2.x breaks the C API they rely on.
NUMPY_VER=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "none")
if [[ "$NUMPY_VER" == "none" ]] || python3 -c "import numpy; assert int(numpy.__version__.split('.')[0]) >= 2" 2>/dev/null; then
    info "  Installing numpy<2 (current: ${NUMPY_VER})..."
    pip3 install "numpy<2"
else
    info "  numpy ${NUMPY_VER} is already <2 -- skipping."
fi

# ---------------------------------------------------------------------------
# 3. Blacklist bcm2835_v4l2
# ---------------------------------------------------------------------------
info "Step 3/6: Blacklisting bcm2835_v4l2 kernel module..."
BLACKLIST_FILE="/etc/modprobe.d/blacklist-bcm2835-v4l2.conf"

if [[ ! -f "$BLACKLIST_FILE" ]]; then
    echo "blacklist bcm2835_v4l2" | sudo tee "$BLACKLIST_FILE" > /dev/null
    info "  Created ${BLACKLIST_FILE}"
    info "  Running update-initramfs (may take ~30s)..."
    sudo update-initramfs -u
    warn "  A reboot is required for this change to take effect."
else
    info "  ${BLACKLIST_FILE} already exists -- skipping."
fi

# Unload the module from the running kernel if it's loaded
if lsmod | grep -q "bcm2835_v4l2"; then
    info "  Unloading bcm2835_v4l2 from running kernel..."
    sudo rmmod bcm2835_v4l2 2>/dev/null || warn "  Could not unload -- reboot to apply."
fi

# ---------------------------------------------------------------------------
# 4. Install libcamera 0.7.0 artifacts to system paths
# ---------------------------------------------------------------------------
info "Step 4/6: Installing libcamera 0.7.0 artifacts..."

if [[ ! -d "$LIBCAM_SRC" ]]; then
    warn "  libcamera source directory not found: ${LIBCAM_SRC}"
    warn "  Skipping libcamera install -- build from source first:"
    warn "    cd ~ && git clone https://git.libcamera.org/libcamera/libcamera.git"
    warn "    cd libcamera && git checkout v0.7.0"
    warn "    meson setup build && ninja -C build"
    warn "    Then copy build artifacts into ${LIBCAM_SRC}/"
else
    # Shared libraries -> /usr/local/lib
    for lib in libcamera.so* libcamera-base.so* libpisp.so*; do
        for f in "${LIBCAM_SRC}"/lib/${lib} 2>/dev/null; do
            [[ -f "$f" ]] && sudo cp -P "$f" /usr/local/lib/
        done
    done

    # GStreamer plugin -> /usr/local/lib/aarch64-linux-gnu/gstreamer-1.0/
    GST_PLUGIN_DEST="/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0"
    sudo mkdir -p "$GST_PLUGIN_DEST"
    if [[ -f "${LIBCAM_SRC}/gstreamer/libgstlibcamera.so" ]]; then
        sudo cp "${LIBCAM_SRC}/gstreamer/libgstlibcamera.so" "$GST_PLUGIN_DEST/"
    fi

    # IPA modules -> /usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
    IPA_DEST="/usr/local/lib/aarch64-linux-gnu/libcamera/ipa"
    sudo mkdir -p "$IPA_DEST"
    for f in "${LIBCAM_SRC}"/ipa/*.so "${LIBCAM_SRC}"/ipa/*.sign; do
        [[ -f "$f" ]] && sudo cp "$f" "$IPA_DEST/"
    done

    # IPA tuning JSON files -> /usr/local/share/libcamera/
    if [[ -d "${LIBCAM_SRC}/share" ]]; then
        sudo cp -r "${LIBCAM_SRC}/share/." /usr/local/share/
    fi

    # Python bindings -> both dist-packages locations
    for PYDIR in /usr/local/lib/python3/dist-packages /usr/local/lib/python3.10/dist-packages; do
        sudo mkdir -p "${PYDIR}/libcamera"
        if [[ -d "${LIBCAM_SRC}/python/libcamera" ]]; then
            sudo cp -r "${LIBCAM_SRC}/python/libcamera/." "${PYDIR}/libcamera/"
        fi
    done

    info "  libcamera artifacts installed."
fi

# ---------------------------------------------------------------------------
# 4b. Symlinks in /usr/local/lib
# ---------------------------------------------------------------------------
# ldconfig expects the canonical .so.MAJOR.MINOR.PATCH → .so.MAJOR.MINOR →
# .so.MAJOR chain. A plain `cp` creates real files instead of symlinks, which
# triggers ldconfig warnings and may cause runtime load failures.
info "Step 4b/6: Creating shared-library symlinks in /usr/local/lib..."

cd /usr/local/lib
for spec in \
    "libcamera.so.0.7.0:libcamera.so.0.7" \
    "libcamera-base.so.0.7.0:libcamera-base.so.0.7" \
    "libpisp.so.1.3.0:libpisp.so.1"
do
    src="${spec%%:*}"
    dst="${spec##*:}"
    if [[ -f "$src" ]] && [[ ! -L "$dst" ]]; then
        sudo ln -sf "$src" "$dst"
        info "  ln -sf $src $dst"
    elif [[ -L "$dst" ]]; then
        info "  $dst already a symlink -- skipping"
    else
        warn "  $src not found -- skipping $dst symlink"
    fi
done
cd - > /dev/null

# ---------------------------------------------------------------------------
# 5. ldconfig
# ---------------------------------------------------------------------------
info "Step 5/6: Configuring dynamic linker..."
LDCONF_FILE="/etc/ld.so.conf.d/libcamera-local.conf"
if [[ ! -f "$LDCONF_FILE" ]]; then
    echo "/usr/local/lib" | sudo tee "$LDCONF_FILE" > /dev/null
    info "  Created ${LDCONF_FILE}"
fi
sudo ldconfig
info "  ldconfig done."

# ---------------------------------------------------------------------------
# 6. ~/.bashrc env vars
# ---------------------------------------------------------------------------
info "Step 6/6: Adding env vars to ~/.bashrc..."
BASHRC="$HOME/.bashrc"
MARKER="# --- Omega-1 libcamera env (added by setup_pi_camera.sh) ---"

if grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
    info "  Env vars already present in ~/.bashrc -- skipping."
else
    cat >> "$BASHRC" <<'EOF'

# --- Omega-1 libcamera env (added by setup_pi_camera.sh) ---
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:${GST_PLUGIN_PATH:-}
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa
export PYTHONPATH=/usr/local/lib/python3/dist-packages:${PYTHONPATH:-}
# --- end Omega-1 libcamera env ---
EOF
    info "  Env vars appended to ~/.bashrc"
    info "  Run: source ~/.bashrc   (or open a new shell)"
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
info "Setup complete."
echo ""
echo "  Verify GStreamer sees libcamerasrc:"
echo "    source ~/.bashrc"
echo "    gst-inspect-1.0 libcamerasrc"
echo ""
echo "  Test camera pipeline directly:"
echo "    gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480,framerate=10/1 ! videoconvert ! autovideosink"
echo ""
echo "  Run the video server:"
echo "    cd ${REPO_ROOT}/servers/robot_controller_backend/video"
echo "    python3 video_server.py"
echo ""

if grep -qF "bcm2835_v4l2" /proc/modules 2>/dev/null || lsmod | grep -q "bcm2835_v4l2"; then
    warn "bcm2835_v4l2 is still loaded. Reboot to fully apply the blacklist."
fi
