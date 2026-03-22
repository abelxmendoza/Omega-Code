#!/usr/bin/env python3
"""
Omega Camera Publisher Node
============================
Publishes camera frames to ROS topics alongside the existing MJPEG stream.

Two operating modes
-------------------
  STANDALONE mode (default when launched via `ros2 run`):
    The node owns its own camera connection and capture loop.
    The MJPEG server (video_server.py) runs as a separate process and owns
    the camera independently via Picamera2 (which supports multi-consumer
    via buffer sharing) or V4L2.

  BRIDGE mode (when integrated into video_server.py):
    video_server.py calls `publisher.publish_frame(frame)` directly.
    The node still spins in a background thread via `init_ros2_publisher()`.
    This is the pattern already used in video/ros2_integration.py --
    this node supersedes that file with proper lifecycle and headers.

Published topics
----------------
  /omega/camera/image_raw            sensor_msgs/Image           (BEST_EFFORT)
  /omega/camera/image_raw/compressed sensor_msgs/CompressedImage (BEST_EFFORT)
  /omega/camera/camera_info          sensor_msgs/CameraInfo      (BEST_EFFORT)

Parameters
----------
  standalone          bool   True    -- open own camera vs. bridge mode
  width               int    640
  height              int    480
  fps                 int    30
  jpeg_quality        int    80      -- JPEG encoding quality (1-100)
  publish_raw         bool   False   -- also publish uncompressed Image
                                        (expensive on Pi -- off by default)
  camera_frame_id     str    'camera_link'
  device_index        int    0       -- V4L2 /dev/videoN index (fallback)

Frame coordinate convention
----------------------------
  frame_id = 'camera_link' -- the optical frame (Z forward, X right, Y down)
  The static transform  base_link -> camera_link  is published by the
  launch file via StaticTransformPublisher.
"""

from __future__ import annotations

import time
import threading
import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header

# cv_bridge is optional -- we fall back to manual encoding if it's missing
try:
    from cv_bridge import CvBridge
    _cv_bridge_ok = True
except ImportError:
    _cv_bridge_ok = False

try:
    import cv2
    import numpy as np
    _cv2_ok = True
except ImportError:
    _cv2_ok = False

# ---------------------------------------------------------------------------
# Camera backend detection
# ---------------------------------------------------------------------------
_picamera2_ok = False
try:
    from picamera2 import Picamera2  # type: ignore
    _picamera2_ok = True
except ImportError:
    pass

# GStreamer + libcamerasrc is the preferred backend on this Pi.
# Requires: libcamera built from source with GStreamer plugin enabled.
# Env vars must be set before launch:
#   export LD_LIBRARY_PATH=$HOME/libcamera/build/src/libcamera:$LD_LIBRARY_PATH
#   export GST_PLUGIN_PATH=$HOME/libcamera/build/src/gstreamer:$GST_PLUGIN_PATH


def _sensor_qos(depth: int = 2) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Camera abstraction -- returns BGR numpy arrays
# ---------------------------------------------------------------------------

class _LibcameraGStreamerSource:
    """
    Capture via GStreamer + libcamerasrc plugin.

    This is the only backend that works reliably with a CSI OV5647 on
    Raspberry Pi when libcamera is built from source.  V4L2 and Picamera2
    failed during bring-up; do not replace this with either of those.

    Pipeline design:
      - queue leaky=downstream: drop old frames, always deliver the newest
      - videoconvert + format=BGR: output matches OpenCV's native format,
        avoiding an extra colour-space conversion inside OpenCV
      - appsink max-buffers=1 drop=true sync=false: low-latency sink
    """

    def __init__(self, width: int, height: int, fps: int,
                 pipeline_override: str = '') -> None:
        if pipeline_override:
            pipeline = pipeline_override
        else:
            pipeline = (
                f'libcamerasrc ! '
                f'video/x-raw,width={width},height={height},framerate={fps}/1 ! '
                f'queue leaky=downstream max-size-buffers=1 ! '
                f'videoconvert ! '
                f'video/x-raw,format=BGR ! '
                f'appsink max-buffers=1 drop=true sync=false'
            )
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(
                'libcamerasrc GStreamer pipeline failed to open. '
                'Ensure GST_PLUGIN_PATH and LD_LIBRARY_PATH point to your '
                'libcamera build and that no other process holds the camera.'
            )

    def read_frame(self) -> Optional['np.ndarray']:
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        try:
            self._cap.release()
        except Exception:
            pass


class _PiCamera2Source:
    def __init__(self, width: int, height: int, fps: int) -> None:
        self._cam = Picamera2()
        config = self._cam.create_video_configuration(
            main={'size': (width, height), 'format': 'RGB888'},
            controls={'FrameRate': float(fps)},
        )
        self._cam.configure(config)
        self._cam.start()
        time.sleep(0.3)   # allow AGC to settle

    def read_frame(self) -> Optional['np.ndarray']:
        """Return BGR frame or None."""
        try:
            rgb = self._cam.capture_array()
            return rgb[:, :, ::-1]   # RGB -> BGR (cv2 convention)
        except Exception:
            return None

    def close(self) -> None:
        try:
            self._cam.stop()
        except Exception:
            pass


class _V4L2Source:
    def __init__(self, width: int, height: int, fps: int, device_index: int) -> None:
        self._cap = cv2.VideoCapture(device_index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS,          fps)
        if not self._cap.isOpened():
            raise RuntimeError(f'/dev/video{device_index} failed to open')

    def read_frame(self) -> Optional['np.ndarray']:
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        try:
            self._cap.release()
        except Exception:
            pass


class _SimSource:
    """Generates a test pattern when no real camera is available."""

    def __init__(self, width: int, height: int) -> None:
        self._w = width
        self._h = height

    def read_frame(self) -> 'np.ndarray':
        frame = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        t = time.monotonic()
        # Scrolling colour bar
        bar = int((t * 40) % self._w)
        frame[:, bar:bar+20, :] = [0, 200, 0]
        cv2.putText(frame, 'OMEGA SIM', (20, self._h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return frame

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class CameraPublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('omega_camera_publisher')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('standalone',          True)
        self.declare_parameter('width',               640)
        self.declare_parameter('height',              480)
        self.declare_parameter('fps',                  10)
        self.declare_parameter('jpeg_quality',         80)
        self.declare_parameter('publish_raw',         False)
        self.declare_parameter('camera_frame_id',     'camera_link')
        self.declare_parameter('device_index',         0)
        # Set this to override the full GStreamer pipeline string.
        # Leave empty to auto-build from width/height/fps.
        self.declare_parameter('gstreamer_pipeline',  '')

        p = self.get_parameter
        self._standalone:       bool = p('standalone').value
        self._width:            int  = p('width').value
        self._height:           int  = p('height').value
        self._fps:              int  = p('fps').value
        self._jpeg_q:           int  = p('jpeg_quality').value
        self._publish_raw:      bool = p('publish_raw').value
        self._frame_id:         str  = p('camera_frame_id').value
        self._device_idx:       int  = p('device_index').value
        self._gst_pipeline_ovr: str  = p('gstreamer_pipeline').value

        # ---- cv_bridge ------------------------------------------------
        self._bridge = CvBridge() if _cv_bridge_ok else None

        # ---- JPEG encode params (OpenCV) ------------------------------
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_q]

        # ---- publishers -----------------------------------------------
        qos = _sensor_qos()
        self._comp_pub  = self.create_publisher(CompressedImage, '/omega/camera/image_raw/compressed', qos)
        self._info_pub  = self.create_publisher(CameraInfo,       '/omega/camera/camera_info',          qos)
        if self._publish_raw:
            self._raw_pub = self.create_publisher(Image, '/omega/camera/image_raw', qos)
        else:
            self._raw_pub = None

        # ---- camera source (standalone only) -------------------------
        self._source = None
        if self._standalone and _cv2_ok:
            self._source = self._open_camera()

        # ---- capture timer (standalone mode) -------------------------
        if self._standalone and self._source is not None:
            period = 1.0 / max(1, self._fps)
            self._capture_timer = self.create_timer(period, self._capture_and_publish)
        else:
            self._capture_timer = None

        # ---- camera info (populated once, reused) --------------------
        self._camera_info_msg = self._build_camera_info_msg()

        self._frame_count = 0
        self._last_log_t  = time.monotonic()

        self.get_logger().info(
            'CameraPublisherNode ready [%dx%d @%dfps standalone=%s raw=%s jpeg_q=%d]',
            self._width, self._height, self._fps,
            self._standalone, self._publish_raw, self._jpeg_q,
        )

    # ------------------------------------------------------------------
    # Camera source selection
    # ------------------------------------------------------------------

    def _open_camera(self):
        # 1. libcamerasrc (preferred on Pi with CSI camera — V4L2 does not work)
        if _cv2_ok:
            try:
                src = _LibcameraGStreamerSource(
                    self._width, self._height, self._fps,
                    pipeline_override=self._gst_pipeline_ovr,
                )
                self.get_logger().info('Camera backend: libcamerasrc (GStreamer)')
                return src
            except Exception as exc:
                self.get_logger().warning(
                    'libcamerasrc GStreamer failed: %s -- trying Picamera2', exc
                )

        # 2. Picamera2 (works if python3-picamera2 is installed)
        if _picamera2_ok:
            try:
                src = _PiCamera2Source(self._width, self._height, self._fps)
                self.get_logger().info('Camera backend: Picamera2')
                return src
            except Exception as exc:
                self.get_logger().warning('Picamera2 failed: %s -- sim mode', exc)

        # 3. Sim (no hardware — test pattern)
        self.get_logger().warning('No real camera available -- publishing test pattern')
        return _SimSource(self._width, self._height)

    # ------------------------------------------------------------------
    # Standalone capture loop (timer callback)
    # ------------------------------------------------------------------

    def _capture_and_publish(self) -> None:
        if self._source is None:
            return
        frame = self._source.read_frame()
        if frame is None:
            return
        self.publish_frame(frame)

    # ------------------------------------------------------------------
    # Public API -- called by video_server.py in bridge mode
    # ------------------------------------------------------------------

    def publish_frame(self, frame: 'np.ndarray') -> None:
        """
        Publish a BGR numpy frame to all active ROS topics.

        This is the bridge-mode entry point -- video_server.py calls this
        from its capture loop so both MJPEG and ROS share the same frame.

        Args:
            frame: BGR ndarray, shape (H, W, 3), dtype uint8
        """
        if frame is None or not _cv2_ok:
            return

        now = self.get_clock().now().to_msg()
        header = Header()
        header.stamp    = now
        header.frame_id = self._frame_id

        # -- Compressed (JPEG) ------------------------------------------
        try:
            ok, buf = cv2.imencode('.jpg', frame, self._encode_params)
            if ok:
                comp_msg = CompressedImage()
                comp_msg.header  = header
                comp_msg.format  = 'jpeg'
                comp_msg.data    = buf.tobytes()
                self._comp_pub.publish(comp_msg)
        except Exception as exc:
            self.get_logger().debug('Compressed encode error: %s', exc)

        # -- Raw Image (optional, expensive on Pi) ----------------------
        if self._raw_pub is not None and self._bridge is not None:
            try:
                # Convert BGR -> RGB for ROS convention
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                raw_msg = self._bridge.cv2_to_imgmsg(rgb, 'rgb8')
                raw_msg.header = header
                self._raw_pub.publish(raw_msg)
            except Exception as exc:
                self.get_logger().debug('Raw image publish error: %s', exc)

        # -- Camera info (same timestamp as frame) ----------------------
        self._camera_info_msg.header = header
        self._info_pub.publish(self._camera_info_msg)

        # -- Rate logging -----------------------------------------------
        self._frame_count += 1
        now_t = time.monotonic()
        if now_t - self._last_log_t >= 10.0:
            fps = self._frame_count / (now_t - self._last_log_t)
            self.get_logger().info('Camera publishing at %.1f fps', fps)
            self._frame_count = 0
            self._last_log_t  = now_t

    # ------------------------------------------------------------------
    # Camera info message (pinhole model -- update with calibration data)
    # ------------------------------------------------------------------

    def _build_camera_info_msg(self) -> CameraInfo:
        """
        Build a CameraInfo message with placeholder intrinsics.

        Replace the K, D, P matrices with values from your calibration
        (run `ros2 run camera_calibration cameracalibrator`).
        """
        msg = CameraInfo()
        msg.header.frame_id = self._frame_id
        msg.width  = self._width
        msg.height = self._height
        msg.distortion_model = 'plumb_bob'

        # Placeholder pinhole intrinsics (fx=fy~focal_length, cx=W/2, cy=H/2)
        # These are approximate -- replace with calibrated values.
        fx = float(self._width)   # rough estimate: fx ~ width pixels
        fy = float(self._width)
        cx = self._width  / 2.0
        cy = self._height / 2.0

        # K: 3x3 intrinsic matrix, row-major
        msg.k = [fx, 0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]

        # D: distortion coefficients [k1, k2, t1, t2, k3]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # R: rectification matrix (identity for monocular)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # P: 3x4 projection matrix
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info('CameraPublisherNode shutting down')
        if self._source is not None:
            try:
                self._source.close()
            except Exception:
                pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Bridge-mode helper  (drop-in replacement for video/ros2_integration.py)
# ---------------------------------------------------------------------------

_bridge_node: Optional[CameraPublisherNode] = None
_bridge_thread: Optional[threading.Thread]  = None


def init_ros2_camera_publisher(
    width: int = 640,
    height: int = 480,
    jpeg_quality: int = 80,
    publish_raw: bool = False,
) -> Optional[CameraPublisherNode]:
    """
    Initialise a bridge-mode camera publisher embedded in video_server.py.

    Call once at server startup.  Returns the node so the caller can invoke
    `node.publish_frame(bgr_frame)` from the capture loop.

    Returns None if rclpy is not available (graceful degradation).
    """
    global _bridge_node, _bridge_thread

    if not rclpy.ok():
        try:
            rclpy.init()
        except Exception:
            return None

    try:
        node = CameraPublisherNode()
        # Override standalone -- we own the camera in video_server.py
        node._standalone = False
        if node._capture_timer:
            node._capture_timer.cancel()

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def _spin():
            try:
                executor.spin()
            except Exception:
                pass

        thread = threading.Thread(target=_spin, name='omega_camera_ros_spin', daemon=True)
        thread.start()

        _bridge_node   = node
        _bridge_thread = thread
        return node

    except Exception as exc:
        logging.getLogger(__name__).warning('Camera ROS publisher init failed: %s', exc)
        return None


def shutdown_ros2_camera_publisher() -> None:
    global _bridge_node
    if _bridge_node is not None:
        try:
            _bridge_node.destroy_node()
        except Exception:
            pass
        _bridge_node = None
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point (standalone mode)
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
