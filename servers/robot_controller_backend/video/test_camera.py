# File: /Omega-Code/servers/robot_controller_backend/video/test_camera.py
"""
Summary
-------
Universal camera probe for Raspberry Pi:

- Tries Picamera2 (CSI ribbon camera) and/or V4L2/OpenCV (/dev/videoN).
- Headless-friendly: saves a snapshot to /tmp and prints diagnostics.
- GUI-friendly: shows a preview window when DISPLAY is available (unless --no-preview).
- Actionable error messages (backend not installed, device busy, permissions, etc.).
- Extra tools: --list-only quickly enumerates cameras and exits.

Usage examples
--------------
# Auto-detect (tries Picamera2 first, then V4L2)
python3 test_camera.py

# Force a backend
python3 test_camera.py --backend picamera2
python3 test_camera.py --backend v4l2 --device /dev/video0

# Change resolution and output
python3 test_camera.py --width 1280 --height 720 --out /tmp/probe.jpg

# Just list cameras and exit
python3 test_camera.py --list-only
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import warnings
import shutil
import subprocess
from typing import Optional, Tuple, Literal

# --- Optional deps ---
try:
    import cv2  # type: ignore
except Exception:
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. V4L2 testing disabled.", ImportWarning)

ColorSpace = Literal["RGB", "BGR"]  # tag frames so we don't double-convert later

# ---------------------------
# Helpers / environment
# ---------------------------
def has_display() -> bool:
    """True if we can open a GUI window (X11/Wayland)."""
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

def run(cmd: list[str]) -> Tuple[int, str, str]:
    """Run a command; return (code, stdout, stderr)."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=8)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except Exception as e:
        return 127, "", str(e)

def list_video_nodes() -> list[str]:
    import glob
    return sorted(glob.glob("/dev/video*"))

def build_has_gstreamer() -> bool:
    """Detect if OpenCV was built with GStreamer (nice for some pipelines)."""
    if cv2 is None:
        return False
    try:
        info = cv2.getBuildInformation()
        return "GStreamer" in info
    except Exception:
        return False

def in_video_group() -> bool:
    """Is current user in the 'video' group (needed for /dev/video* access)?"""
    code, out, _ = run(["id", "-nG"])
    return (code == 0) and ("video" in out.split())

def who_uses_device(dev: str) -> str:
    """Return a hint of which process is holding /dev/videoN (if any)."""
    if shutil.which("fuser"):
        code, out, _ = run(["fuser", "-v", dev])
        if code == 0 and out:
            return f"(in use) fuser: {out}"
    if shutil.which("lsof"):
        code, out, _ = run(["lsof", dev])
        if code == 0 and out:
            return f"(in use)\n{out}"
    return ""

# ---------------------------
# Picamera2 backend (CSI)
# ---------------------------
def try_picamera2(width: int, height: int) -> Optional[Tuple["object", ColorSpace]]:
    """
    Capture one frame via Picamera2 (CSI ribbon cameras).
    Returns (frame, colorspace_tag) or None.
    Colorspace tag is 'RGB' (Picamera2 produces RGB arrays).
    """
    try:
        from picamera2 import Picamera2
    except Exception as e:
        print(f"❌ Picamera2 not available: {e}")
        return None

    try:
        picam = Picamera2()
        # Most sensors support these preview sizes; caller can override via args.
        cfg = picam.create_preview_configuration(main={"format": "RGB888", "size": (width, height)})
        picam.configure(cfg)
        picam.start()
        time.sleep(0.3)  # small warmup
        rgb = picam.capture_array("main")  # RGB888 numpy array
        picam.stop()
        if getattr(rgb, "size", 0) <= 0:
            print("❌ Picamera2 returned empty frame.")
            return None
        return rgb, "RGB"
    except Exception as e:
        print(f"❌ Picamera2 capture failed: {e}")
        return None

# ---------------------------
# V4L2 backend (USB / /dev/videoN)
# ---------------------------
def try_v4l2(device: str, width: int, height: int, fps: int = 30, fourcc: str = "MJPG") -> Optional[Tuple["object", ColorSpace]]:
    """
    Capture one frame via OpenCV VideoCapture (V4L2).
    Returns (frame, 'BGR') or None.
    """
    if cv2 is None:
        print("❌ OpenCV not installed; cannot test V4L2.")
        return None

    # Prefer CAP_V4L2 on Linux
    api = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else 0
    cap = cv2.VideoCapture(device, api)
    if not cap or not cap.isOpened():
        # Try a GStreamer fallback if OpenCV has GStreamer support
        if build_has_gstreamer():
            gst = f"v4l2src device={device} ! image/jpeg,framerate={fps}/1 ! jpegdec ! videoconvert ! appsink"
            cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if not cap or not cap.isOpened():
                print(f"❌ Could not open {device} via V4L2 or GStreamer.")
                return None
        else:
            print(f"❌ Could not open {device} via V4L2 (and no GStreamer build in OpenCV).")
            hint = who_uses_device(device)
            if hint:
                print("   " + hint)
            return None

    # Best-effort hints
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        cap.set(cv2.CAP_PROP_FPS,          float(fps))
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    except Exception:
        pass

    # Grab a couple of frames to stabilize
    t0 = time.time()
    frame = None
    while time.time() - t0 < 2.0:
        ok, fr = cap.read()
        if ok and fr is not None:
            frame = fr
            break
        time.sleep(0.02)

    cap.release()

    if frame is None:
        print("❌ V4L2: failed to read a frame (device busy or unsupported format).")
        hint = who_uses_device(device)
        if hint:
            print("   " + hint)
        return None

    # Normalize to 3-channel BGR
    try:
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    except Exception:
        pass
    return frame, "BGR"

# ---------------------------
# Main CLI
# ---------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description="Probe Raspberry Pi camera (Picamera2 or V4L2).")
    parser.add_argument("--backend", choices=["auto", "picamera2", "v4l2"],
                        default=os.getenv("CAMERA_BACKEND", "auto"),
                        help="Which backend to use.")
    parser.add_argument("--device", default="/dev/video0", help="V4L2 device path when using --backend v4l2.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--out", default="/tmp/camera_probe.jpg", help="Snapshot output path (headless).")
    parser.add_argument("--no-preview", action="store_true", help="Do not open a preview window even if GUI is available.")
    parser.add_argument("--list-only", action="store_true", help="List detected cameras and exit.")
    args = parser.parse_args()

    print("🔎 Camera probe starting…")
    print(f"   Backend: {args.backend}")
    print(f"   Size: {args.width}x{args.height}")
    print(f"   OpenCV: {'yes' if cv2 is not None else 'no'} (GStreamer: {'yes' if build_has_gstreamer() else 'no'})")
    print(f"   User in 'video' group: {'yes' if in_video_group() else 'no'}")

    # V4L2 inventory (helpful in all modes)
    nodes = list_video_nodes()
    print(f"   V4L2 nodes: {', '.join(nodes) if nodes else '(none)'}")
    if shutil.which("v4l2-ctl"):
        code, out, _ = run(["v4l2-ctl", "--list-devices"])
        if out:
            print("   v4l2-ctl --list-devices:\n" + "\n".join("     " + line for line in out.splitlines()))
    else:
        print("   (hint) install v4l-utils for richer V4L2 diagnostics: sudo apt-get install -y v4l-utils")

    if args.list_only:
        # Also try to list CSI cameras via rpicam-hello (non-fatal if missing)
        if shutil.which("rpicam-hello"):
            _, out, _ = run(["rpicam-hello", "--list-cameras"])
            if out:
                print("\nCSI cameras (rpicam-hello --list-cameras):\n" + out)
        return 0

    # Try Picamera2 first (CSI), then V4L2 (USB) in auto mode
    backends = [args.backend] if args.backend != "auto" else ["picamera2", "v4l2"]

    frame: Optional["object"] = None
    cs: ColorSpace = "BGR"
    used_backend: Optional[str] = None

    for be in backends:
        if be == "picamera2":
            res = try_picamera2(args.width, args.height)
            if res is not None:
                frame, cs = res
                used_backend = "picamera2"
                break
        elif be == "v4l2":
            res = try_v4l2(args.device, args.width, args.height, fps=args.fps)
            if res is not None:
                frame, cs = res
                used_backend = "v4l2"
                break

    if frame is None:
        print("🚫 No frame captured. Things to check:")
        print("  • For CSI ribbon cameras: ensure Picamera2 is installed and the sensor is detected:")
        print("      rpicam-hello --list-cameras")
        print("  • For USB webcams: confirm the node exists and isn’t busy:")
        print("      ls -l /dev/video*   &&   v4l2-ctl --list-devices")
        print("  • If another process is using the camera, stop it (rpicam-*, previous server, etc.).")
        print("  • If GUI preview fails on headless, re-run with --no-preview (we’ll save a still).")
        return 2

    print(f"✅ Captured a frame via {used_backend} ({cs})")

    # Headless -> save; GUI -> preview window (unless --no-preview)
    if (cv2 is not None) and has_display() and not args.no_preview:
        try:
            disp = frame
            # OpenCV expects BGR; convert if we got RGB from Picamera2
            if cs == "RGB":
                disp = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow(f"Camera ({used_backend})", disp)
            print("🎥 Preview open. Press 'q' to quit.")
            while True:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"⚠️ Preview failed: {e} (saving to {args.out} instead)")
            try:
                if cv2 is not None:
                    out_img = frame
                    if cs == "RGB":
                        out_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(args.out, out_img)
                print(f"💾 Saved snapshot: {args.out}")
            except Exception as e2:
                print(f"❌ Save failed: {e2}")
                return 3
    else:
        # Headless or preview suppressed: save snapshot
        try:
            if cv2 is not None:
                out_img = frame
                if cs == "RGB":
                    out_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(args.out, out_img)
            else:
                # Best-effort write without OpenCV: try Pillow if present
                try:
                    from PIL import Image  # type: ignore
                    img = Image.fromarray(frame)  # type: ignore[arg-type]
                    img.save(args.out)
                except Exception as e:
                    print(f"❌ Pillow not available or save failed: {e}")
                    return 4
            print(f"💾 Saved snapshot: {args.out}")
        except Exception as e:
            print(f"❌ Save failed: {e}")
            return 3

    return 0

if __name__ == "__main__":
    sys.exit(main())
