#!/usr/bin/env python3
"""
camera_debug_node.py
====================
Subscribes to /camera/image_raw and prints a summary every 2 seconds:
  - resolution and encoding (logged once on first frame)
  - approximate FPS (frames received in the last 2 s interval)

Usage
-----
  ros2 run omega_robot camera_debug

Topics
------
  Subscribed:
    /camera/image_raw   sensor_msgs/msg/Image
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def _be_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class CameraDebugNode(Node):

    def __init__(self) -> None:
        super().__init__('camera_debug')

        self._frame_count: int = 0
        self._first_frame: bool = True
        self._last_report_t: float = time.monotonic()

        self._sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._on_image,
            _be_qos(depth=5),
        )

        self.create_timer(2.0, self._report)

        self.get_logger().info(
            'CameraDebugNode ready — waiting for /camera/image_raw ...'
        )

    def _on_image(self, msg: Image) -> None:
        self._frame_count += 1
        if self._first_frame:
            self._first_frame = False
            self.get_logger().info(
                f'First frame: {msg.width}x{msg.height}  '
                f'encoding={msg.encoding}  '
                f'step={msg.step}'
            )

    def _report(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_report_t
        fps = self._frame_count / elapsed if elapsed > 0 else 0.0

        if self._frame_count == 0:
            self.get_logger().info('No frames received on /camera/image_raw')
        else:
            self.get_logger().info(
                f'Camera: {fps:.1f} fps  ({self._frame_count} frames in {elapsed:.1f}s)'
            )

        self._frame_count = 0
        self._last_report_t = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraDebugNode()
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
