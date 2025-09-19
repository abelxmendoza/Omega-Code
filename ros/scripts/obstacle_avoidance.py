#!/usr/bin/env python3
"""ROS node for real-time obstacle avoidance using ultrasonic and camera data."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


@dataclass
class CameraObservation:
    """Lightweight container describing the obstacle density detected in each region."""

    left_density: float
    center_density: float
    right_density: float

    def obstacle_bias(self) -> float:
        """Return signed bias: >0 when the right side has more obstacles (turn left)."""

        return float(self.right_density - self.left_density)

    @property
    def total_density(self) -> float:
        """Total proportion of pixels flagged as obstacle across all regions."""

        return float(self.left_density + self.center_density + self.right_density)


class ObstacleAvoidanceNode:
    """Fuse ultrasonic distance and camera data to keep the robot away from obstacles."""

    def __init__(self) -> None:
        try:
            self.node_name = rospy.get_name()
        except rospy.ROSException:
            self.node_name = "obstacle_avoidance"

        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Tunable parameters with sensible defaults.
        self.ultrasonic_topic = rospy.get_param("~ultrasonic_topic", "ultrasonic/distance")
        self.camera_topic = rospy.get_param("~camera_topic", "camera/image")
        self.safe_distance = float(rospy.get_param("~safe_distance_cm", 35.0))
        self.stop_distance = float(rospy.get_param("~stop_distance_cm", 20.0))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.3))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.0))
        self.camera_weight = float(rospy.get_param("~camera_weight", 0.6))
        self.camera_center_threshold = float(rospy.get_param("~center_density_threshold", 0.06))
        self.ultrasonic_samples = max(1, int(rospy.get_param("~ultrasonic_samples", 5)))
        self.camera_timeout = rospy.Duration(float(rospy.get_param("~camera_timeout", 1.5)))

        # Rolling buffer to smooth noisy ultrasonic readings.
        self.distance_buffer: deque[float] = deque(maxlen=self.ultrasonic_samples)
        self.latest_distance: Optional[float] = None

        # Camera observation bookkeeping.
        self._last_camera_stamp = rospy.Time(0)
        self._latest_camera_obs: Optional[CameraObservation] = None

        rospy.Subscriber(
            self.ultrasonic_topic,
            Float32,
            self.ultrasonic_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.camera_topic,
            Image,
            self.camera_callback,
            queue_size=1,
        )

        # Control loop running at 10 Hz.
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("[%s] Obstacle avoidance node initialized.", self.node_name)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def ultrasonic_callback(self, msg: Float32) -> None:
        """Handle new ultrasonic measurements, filtering out invalid data."""

        distance = float(msg.data)
        if not math.isfinite(distance) or distance < 0:
            rospy.logwarn_throttle(5.0, "[%s] Invalid ultrasonic reading: %.2f", self.node_name, distance)
            return

        self.distance_buffer.append(distance)
        self.latest_distance = sum(self.distance_buffer) / len(self.distance_buffer)
        rospy.logdebug("[%s] Smoothed distance: %.2f cm", self.node_name, self.latest_distance)

    def camera_callback(self, msg: Image) -> None:
        """Convert camera images to obstacle density observations."""

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "[%s] CV bridge error: %s", self.node_name, exc)
            return

        self._last_camera_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self._latest_camera_obs = self._analyze_camera_frame(cv_image)

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------
    def control_loop(self, _event: rospy.TimerEvent) -> None:
        """Publish velocity commands that keep the robot clear of obstacles."""

        if rospy.is_shutdown():
            return

        twist = Twist()
        distance = self.latest_distance
        camera_obs = self._current_camera_observation()

        # Safety first: stop immediately when the ultrasonic sensor reports danger.
        if distance is not None and distance <= self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = self._emergency_turn(camera_obs)
            self.cmd_pub.publish(twist)
            rospy.loginfo_throttle(1.0, "[%s] Emergency stop: obstacle %.2f cm away.", self.node_name, distance)
            return

        # Derive turning command from camera observations when available.
        angular_z = self._camera_based_turn(camera_obs)

        # Adjust forward speed based on available clearance.
        twist.linear.x = self._compute_linear_speed(distance, camera_obs)
        twist.angular.z = angular_z

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _current_camera_observation(self) -> Optional[CameraObservation]:
        """Return the latest camera observation if it is still fresh."""

        if self._latest_camera_obs is None:
            return None

        if rospy.Time.now() - self._last_camera_stamp > self.camera_timeout:
            rospy.logwarn_throttle(
                5.0,
                "[%s] Camera data timeout (%.2f s).",
                self.node_name,
                self.camera_timeout.to_sec(),
            )
            return None

        return self._latest_camera_obs

    def _compute_linear_speed(
        self,
        distance: Optional[float],
        camera_obs: Optional[CameraObservation],
    ) -> float:
        """Determine forward speed based on distance clearance and visual cues."""

        # Default to zero speed when sensor data is missing to stay safe.
        if distance is None and camera_obs is None:
            rospy.logwarn_throttle(5.0, "[%s] No sensor data; holding position.", self.node_name)
            return 0.0

        if distance is None:
            # Rely on camera only: slow down proportionally to the detected obstacle density.
            density = camera_obs.total_density if camera_obs else 0.0
            scaling = max(0.0, 1.0 - min(1.0, density * 2.0))
            return self.max_linear_speed * scaling

        if distance >= self.safe_distance:
            return self.max_linear_speed

        if distance <= self.stop_distance:
            return 0.0

        # Linearly scale between stop_distance and safe_distance.
        ratio = (distance - self.stop_distance) / max(1e-6, self.safe_distance - self.stop_distance)
        ratio = min(max(ratio, 0.0), 1.0)

        if camera_obs and camera_obs.center_density > self.camera_center_threshold:
            # Additional slowdown when the camera sees something ahead.
            ratio *= 0.5

        return self.max_linear_speed * ratio

    def _camera_based_turn(self, camera_obs: Optional[CameraObservation]) -> float:
        """Compute the angular velocity suggested by the camera observation."""

        if camera_obs is None:
            return 0.0

        bias = camera_obs.obstacle_bias()
        turn = bias * self.max_angular_speed * self.camera_weight

        if camera_obs.center_density > self.camera_center_threshold:
            # Encourage stronger turning when something is directly ahead.
            adjustment = self.max_angular_speed * 0.5
            turn += adjustment if turn >= 0 else -adjustment

        return float(np.clip(turn, -self.max_angular_speed, self.max_angular_speed))

    def _emergency_turn(self, camera_obs: Optional[CameraObservation]) -> float:
        """Return a deterministic turn direction during emergency stops."""

        if camera_obs is None:
            return self.max_angular_speed

        if camera_obs.left_density < camera_obs.right_density:
            return self.max_angular_speed
        return -self.max_angular_speed

    def _analyze_camera_frame(self, image: np.ndarray) -> CameraObservation:
        """Extract obstacle density in left/center/right regions using edge detection."""

        # Focus on the lower half of the frame where nearby obstacles appear.
        height = image.shape[0]
        roi = image[height // 2 :, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 75, 200)

        # Split the region of interest into thirds.
        h, w = edges.shape
        third = w // 3 or 1
        left_region = edges[:, :third]
        center_region = edges[:, third : 2 * third]
        right_region = edges[:, 2 * third :]

        def _density(region: np.ndarray) -> float:
            # Canny edges are 0 or 255; normalize counts to [0, 1].
            total_pixels = float(region.size)
            if total_pixels <= 0:
                return 0.0
            return float(np.count_nonzero(region)) / total_pixels

        observation = CameraObservation(
            left_density=_density(left_region),
            center_density=_density(center_region),
            right_density=_density(right_region),
        )

        rospy.logdebug(
            "[%s] Camera densities -> left: %.3f center: %.3f right: %.3f",
            self.node_name,
            observation.left_density,
            observation.center_density,
            observation.right_density,
        )

        return observation


def main() -> None:
    """Entry point for the obstacle avoidance node."""

    try:
        rospy.init_node("obstacle_avoidance", anonymous=False)
        ObstacleAvoidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Obstacle avoidance node interrupted.")
    except Exception as exc:  # pylint: disable=broad-except
        # Catch-all to surface unexpected issues instead of silently crashing.
        rospy.logerr("Fatal error in obstacle avoidance node: %s", exc)
        raise


if __name__ == "__main__":
    main()
