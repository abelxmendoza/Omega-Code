#!/usr/bin/env python3
"""
omega_hybrid.launch.py -- Full hybrid stack on Raspberry Pi
=============================================================
Launches the complete ROS2 layer.  Non-ROS services (FastAPI, Go lighting
server, MJPEG stream) are managed separately by OmegaOS / systemd.

Run on Pi:
    ros2 launch omega_bringup omega_hybrid.launch.py

Common overrides:
    sim_mode:=true               -- no hardware I/O
    launch_camera:=false         -- skip camera node (MJPEG server handles it)
    wheel_base:=0.18             -- robot-specific geometry
"""

from __future__ import annotations
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument('sim_mode',           default_value='false'),
        DeclareLaunchArgument('wheel_base',         default_value='0.20'),
        DeclareLaunchArgument('wheel_radius',       default_value='0.05'),
        DeclareLaunchArgument('max_pwm',            default_value='4095'),
        DeclareLaunchArgument('max_rpm',            default_value='300.0'),
        DeclareLaunchArgument('watchdog_timeout',   default_value='0.50'),
        DeclareLaunchArgument('ramp_type',          default_value='linear'),
        DeclareLaunchArgument('accel_rate',         default_value='4000.0'),
        DeclareLaunchArgument('decel_rate',         default_value='6000.0'),
        DeclareLaunchArgument('thermal_enabled',    default_value='true'),
        DeclareLaunchArgument('launch_camera',      default_value='true'),
        DeclareLaunchArgument('camera_width',       default_value='640'),
        DeclareLaunchArgument('camera_height',      default_value='480'),
        DeclareLaunchArgument('camera_fps',         default_value='10',
                              description='Capture framerate for libcamerasrc pipeline'),
        DeclareLaunchArgument('ultrasonic_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('line_rate_hz',       default_value='20.0'),
        DeclareLaunchArgument('battery_rate_hz',    default_value='1.0'),
        # Physical sensor GPIO pins
        DeclareLaunchArgument('pin_left',           default_value='14'),
        DeclareLaunchArgument('pin_center',         default_value='15'),
        DeclareLaunchArgument('pin_right',          default_value='23'),
        # Servo / pan-tilt parameters
        DeclareLaunchArgument('launch_servo',       default_value='true'),
        DeclareLaunchArgument('yaw_ch',             default_value='8'),
        DeclareLaunchArgument('pitch_ch',           default_value='9'),
        DeclareLaunchArgument('yaw_center_us',      default_value='1500'),
        DeclareLaunchArgument('pitch_center_us',    default_value='1500'),
        DeclareLaunchArgument('yaw_min_us',         default_value='1000'),
        DeclareLaunchArgument('yaw_max_us',         default_value='2000'),
        DeclareLaunchArgument('pitch_min_us',       default_value='1300'),
        DeclareLaunchArgument('pitch_max_us',       default_value='1700'),
        DeclareLaunchArgument('servo_scale_us',     default_value='15.0'),
        # TF geometry (metres) -- adjust to your robot
        # camera: x=forward, z=up from base_link origin
        DeclareLaunchArgument('camera_x',           default_value='0.08'),
        DeclareLaunchArgument('camera_y',           default_value='0.0'),
        DeclareLaunchArgument('camera_z',           default_value='0.10'),
        # ultrasonic: front-mounted at base height
        DeclareLaunchArgument('us_x',               default_value='0.12'),
        DeclareLaunchArgument('us_y',               default_value='0.0'),
        DeclareLaunchArgument('us_z',               default_value='0.02'),
        # Obstacle avoidance
        DeclareLaunchArgument('launch_avoidance',   default_value='true',
                              description='Launch ultrasonic obstacle avoidance node'),
        DeclareLaunchArgument('warn_distance_m',    default_value='0.50',
                              description='Slow-down zone start distance (m)'),
        DeclareLaunchArgument('stop_distance_m',    default_value='0.25',
                              description='Hard-stop / pivot trigger distance (m)'),
        DeclareLaunchArgument('pivot_speed',        default_value='0.70',
                              description='Angular velocity during pivot (normalised rad/s)'),
        DeclareLaunchArgument('pivot_duration_s',   default_value='1.20',
                              description='Duration of each pivot burst (seconds)'),
    ]

    sim         = LaunchConfiguration('sim_mode')
    launch_cam  = LaunchConfiguration('launch_camera')

    # ------------------------------------------------------------------
    # Motor controller
    # ------------------------------------------------------------------
    motor_node = Node(
        package='omega_robot',
        executable='motor_controller',
        name='omega_motor_controller',
        output='screen',
        parameters=[{
            'sim_mode':         sim,
            'wheel_base':       LaunchConfiguration('wheel_base'),
            'wheel_radius':     LaunchConfiguration('wheel_radius'),
            'max_pwm':          LaunchConfiguration('max_pwm'),
            'max_rpm':          LaunchConfiguration('max_rpm'),
            'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            'ramp_type':        LaunchConfiguration('ramp_type'),
            'accel_rate':       LaunchConfiguration('accel_rate'),
            'decel_rate':       LaunchConfiguration('decel_rate'),
            'thermal_enabled':  LaunchConfiguration('thermal_enabled'),
            'ramp_enabled':     True,
        }],
    )

    # ------------------------------------------------------------------
    # Sensor node
    # ------------------------------------------------------------------
    sensor_node = Node(
        package='omega_robot',
        executable='sensor_node',
        name='omega_sensor_node',
        output='screen',
        parameters=[{
            'sim_mode':           sim,
            'ultrasonic_rate_hz': LaunchConfiguration('ultrasonic_rate_hz'),
            'line_rate_hz':       LaunchConfiguration('line_rate_hz'),
            'battery_rate_hz':    LaunchConfiguration('battery_rate_hz'),
            'pin_left':           LaunchConfiguration('pin_left'),
            'pin_center':         LaunchConfiguration('pin_center'),
            'pin_right':          LaunchConfiguration('pin_right'),
        }],
    )

    # ------------------------------------------------------------------
    # Camera node (optional)
    # Backend priority: libcamerasrc (GStreamer) > Picamera2 > sim
    # V4L2 is NOT used -- it fails on this Pi's CSI OV5647 camera.
    # Requires env vars pointing at the local libcamera build:
    #   LD_LIBRARY_PATH=$HOME/libcamera/build/src/libcamera:...
    #   GST_PLUGIN_PATH=$HOME/libcamera/build/src/gstreamer:...
    # ------------------------------------------------------------------
    camera_node = Node(
        package='omega_robot',
        executable='camera_publisher_node',
        name='omega_camera_publisher',
        output='screen',
        condition=IfCondition(launch_cam),
        parameters=[{
            'standalone':      True,
            'width':           LaunchConfiguration('camera_width'),
            'height':          LaunchConfiguration('camera_height'),
            'fps':             LaunchConfiguration('camera_fps'),
            'jpeg_quality':    80,
            'publish_raw':     False,
            'camera_frame_id': 'camera_link',
            # Leave gstreamer_pipeline empty to auto-build from width/height/fps.
            # Override example:
            #   'gstreamer_pipeline': 'libcamerasrc ! video/x-raw,...'
            'gstreamer_pipeline': '',
        }],
    )

    # ------------------------------------------------------------------
    # Servo controller (pan-tilt gimbal on channels 8 and 9)
    # ------------------------------------------------------------------
    servo_node = Node(
        package='omega_robot',
        executable='servo_controller',
        name='omega_servo_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_servo')),
        parameters=[{
            'sim_mode':         sim,
            'yaw_ch':           LaunchConfiguration('yaw_ch'),
            'pitch_ch':         LaunchConfiguration('pitch_ch'),
            'yaw_center_us':    LaunchConfiguration('yaw_center_us'),
            'pitch_center_us':  LaunchConfiguration('pitch_center_us'),
            'yaw_min_us':       LaunchConfiguration('yaw_min_us'),
            'yaw_max_us':       LaunchConfiguration('yaw_max_us'),
            'pitch_min_us':     LaunchConfiguration('pitch_min_us'),
            'pitch_max_us':     LaunchConfiguration('pitch_max_us'),
            'scale_us':         LaunchConfiguration('servo_scale_us'),
            'rate_hz':          20.0,
            'deadzone':         0.10,
        }],
    )

    # ------------------------------------------------------------------
    # Ultrasonic obstacle avoidance (cmd_vel mux)
    # Subscribes to /cmd_vel_in from bridge/teleop, publishes to /cmd_vel.
    # When disabled it passes commands through unchanged.
    # ------------------------------------------------------------------
    avoidance_node = Node(
        package='omega_robot',
        executable='obstacle_avoidance_node',
        name='omega_ultrasonic_avoidance',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_avoidance')),
        parameters=[{
            'warn_distance_m':  LaunchConfiguration('warn_distance_m'),
            'stop_distance_m':  LaunchConfiguration('stop_distance_m'),
            'pivot_speed':      LaunchConfiguration('pivot_speed'),
            'pivot_duration_s': LaunchConfiguration('pivot_duration_s'),
            'sensor_timeout_s': 1.0,
            'control_rate_hz':  20.0,
        }],
    )

    # ------------------------------------------------------------------
    # Capability detector (publishes /omega/capabilities)
    # ------------------------------------------------------------------
    capability_node = Node(
        package='omega_robot',
        executable='system_capabilities',
        name='omega_capability_detector',
        output='screen',
        parameters=[{'publish_interval': 5.0}],
    )

    # ------------------------------------------------------------------
    # Static TF publishers
    # base_link --> camera_link
    # ------------------------------------------------------------------
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_camera',
        output='screen',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            '0.0', '0.0', '0.0',
            'base_link', 'camera_link',
        ],
    )

    # base_link --> ultrasonic_front
    tf_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_ultrasonic',
        output='screen',
        arguments=[
            LaunchConfiguration('us_x'),
            LaunchConfiguration('us_y'),
            LaunchConfiguration('us_z'),
            '0.0', '0.0', '0.0',
            'base_link', 'ultrasonic_front',
        ],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg=['[omega_hybrid] Starting full hybrid stack (sim=', sim, ')']),
        motor_node,
        sensor_node,
        avoidance_node,
        camera_node,
        servo_node,
        capability_node,
        tf_camera,
        tf_ultrasonic,
    ])
