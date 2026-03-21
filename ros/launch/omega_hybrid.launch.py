#!/usr/bin/env python3
"""
Omega Hybrid Launch File
==========================
Launches the ROS2 layer of the Omega-1 hybrid architecture.
The existing WebSocket/REST/Go services run separately and are NOT
started here -- this file manages only the ROS node graph.

Node graph launched
-------------------
  omega_motor_controller     -- /cmd_vel -> PCA9685 PWM + /odom
  omega_sensor_node          -- HC-SR04, line tracking, battery -> ROS topics
  omega_camera_publisher     -- camera -> /omega/camera/... topics
  omega_capability_detector  -- publishes /omega/capabilities profile
  static_tf_base_camera      -- base_link -> camera_link (fixed mount)
  static_tf_base_ultrasonic  -- base_link -> ultrasonic_front (fixed mount)

Usage
-----
  # On Raspberry Pi (hardware):
  ros2 launch omega_robot omega_hybrid.launch.py

  # Simulation (no hardware writes, synthetic sensor data):
  ros2 launch omega_robot omega_hybrid.launch.py sim_mode:=true

  # Custom wheel geometry:
  ros2 launch omega_robot omega_hybrid.launch.py wheel_base:=0.18 wheel_radius:=0.04

  # Disable camera (save CPU on Pi when MJPEG server is running):
  ros2 launch omega_robot omega_hybrid.launch.py launch_camera:=false

  # High-quality camera for laptop/Jetson:
  ros2 launch omega_robot omega_hybrid.launch.py camera_width:=1280 camera_height:=720

Launch arguments
----------------
  sim_mode            bool   false    -- no-op all hardware
  wheel_base          float  0.20     -- metres
  wheel_radius        float  0.05     -- metres
  max_rpm             float  300.0    -- for odometry RPM estimation
  watchdog_timeout    float  0.50     -- seconds before motor safe-stop
  ramp_type           str    linear   -- linear|exponential|s_curve
  accel_rate          float  150.0    -- PWM units/s
  decel_rate          float  200.0    -- PWM units/s
  launch_camera       bool   true     -- include camera publisher node
  camera_width        int    640
  camera_height       int    480
  camera_fps          int    30
  camera_jpeg_quality int    80
  publish_raw_images  bool   false    -- also publish sensor_msgs/Image
  ultrasonic_rate_hz  float  10.0
  line_rate_hz        float  20.0

TF tree produced
----------------
  odom
   └─ base_link          (dynamic, from motor_controller_node odometry)
       ├─ camera_link     (static, 0.08 m forward, 0.10 m up, 0 deg tilt)
       └─ ultrasonic_front (static, 0.12 m forward, 0.0 m up)

Adjust the static transform values to match your physical robot geometry.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os


# ---------------------------------------------------------------------------
# Helper: resolve the project root for PYTHONPATH injection
# ---------------------------------------------------------------------------

def _project_root() -> str:
    """
    Resolve the Omega-Code project root at launch time.
    Assumes: ros/launch/omega_hybrid.launch.py
    Project root is 2 levels up.
    """
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(launch_dir, '..', '..'))


def generate_launch_description() -> LaunchDescription:

    project_root = _project_root()
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    augmented_pythonpath = f'{project_root}:{existing_pythonpath}'.rstrip(':')

    # ---- launch arguments ------------------------------------------------
    args = [
        DeclareLaunchArgument('sim_mode',            default_value='false',
            description='No-op all hardware I/O'),
        DeclareLaunchArgument('wheel_base',           default_value='0.20',
            description='Distance between wheel centrelines in metres'),
        DeclareLaunchArgument('wheel_radius',         default_value='0.05',
            description='Wheel radius in metres'),
        DeclareLaunchArgument('max_rpm',              default_value='300.0',
            description='Max motor RPM (for dead-reckoning odometry)'),
        DeclareLaunchArgument('watchdog_timeout',     default_value='0.50',
            description='Seconds before watchdog stops motors'),
        DeclareLaunchArgument('thermal_enabled',      default_value='true'),
        DeclareLaunchArgument('ramp_type',            default_value='linear'),
        DeclareLaunchArgument('accel_rate',           default_value='150.0'),
        DeclareLaunchArgument('decel_rate',           default_value='200.0'),
        DeclareLaunchArgument('launch_camera',        default_value='true',
            description='Include camera publisher node'),
        DeclareLaunchArgument('camera_width',         default_value='640'),
        DeclareLaunchArgument('camera_height',        default_value='480'),
        DeclareLaunchArgument('camera_fps',           default_value='30'),
        DeclareLaunchArgument('camera_jpeg_quality',  default_value='80'),
        DeclareLaunchArgument('publish_raw_images',   default_value='false'),
        DeclareLaunchArgument('ultrasonic_rate_hz',   default_value='10.0'),
        DeclareLaunchArgument('line_rate_hz',         default_value='20.0'),
        DeclareLaunchArgument('battery_rate_hz',      default_value='1.0'),
    ]

    # ---- convenience references -----------------------------------------
    sim            = LaunchConfiguration('sim_mode')
    wheel_base     = LaunchConfiguration('wheel_base')
    wheel_radius   = LaunchConfiguration('wheel_radius')
    max_rpm        = LaunchConfiguration('max_rpm')
    wd_timeout     = LaunchConfiguration('watchdog_timeout')
    thermal_en     = LaunchConfiguration('thermal_enabled')
    ramp_type      = LaunchConfiguration('ramp_type')
    accel          = LaunchConfiguration('accel_rate')
    decel          = LaunchConfiguration('decel_rate')
    launch_cam     = LaunchConfiguration('launch_camera')
    cam_w          = LaunchConfiguration('camera_width')
    cam_h          = LaunchConfiguration('camera_height')
    cam_fps        = LaunchConfiguration('camera_fps')
    cam_q          = LaunchConfiguration('camera_jpeg_quality')
    pub_raw        = LaunchConfiguration('publish_raw_images')
    us_rate        = LaunchConfiguration('ultrasonic_rate_hz')
    line_rate      = LaunchConfiguration('line_rate_hz')
    bat_rate       = LaunchConfiguration('battery_rate_hz')

    # ---- global parameter (sets sim_mode across all nodes) ---------------
    global_sim = SetParameter(name='sim_mode', value=sim)

    # ---- motor controller node ------------------------------------------
    motor_node = Node(
        package='omega_robot',
        executable='motor_controller',
        name='omega_motor_controller',
        output='screen',
        parameters=[{
            'sim_mode':        sim,
            'wheel_base':      wheel_base,
            'wheel_radius':    wheel_radius,
            'max_rpm':         max_rpm,
            'watchdog_timeout': wd_timeout,
            'thermal_enabled': thermal_en,
            'ramp_enabled':    True,
            'ramp_type':       ramp_type,
            'accel_rate':      accel,
            'decel_rate':      decel,
            'max_pwm':         4095,
        }],
        # No PYTHONPATH needed: pip install -e /path/to/Omega-Code/ handled it
        remappings=[],
    )

    # ---- sensor node ----------------------------------------------------
    sensor_node = Node(
        package='omega_robot',
        executable='sensor_node',
        name='omega_sensor_node',
        output='screen',
        parameters=[{
            'sim_mode':           sim,
            'ultrasonic_rate_hz': us_rate,
            'line_rate_hz':       line_rate,
            'battery_rate_hz':    bat_rate,
            # Line tracking GPIO pins -- override from environment if needed
            'pin_left':           int(os.environ.get('PIN_LEFT',   '14')),
            'pin_center':         int(os.environ.get('PIN_CENTER', '15')),
            'pin_right':          int(os.environ.get('PIN_RIGHT',  '23')),
        }],
        # No PYTHONPATH needed: pip install -e /path/to/Omega-Code/ handled it
    )

    # ---- camera publisher node ------------------------------------------
    camera_node = Node(
        package='omega_robot',
        executable='camera_publisher_node',
        name='omega_camera_publisher',
        output='screen',
        condition=IfCondition(launch_cam),
        parameters=[{
            'standalone':       True,
            'width':            cam_w,
            'height':           cam_h,
            'fps':              cam_fps,
            'jpeg_quality':     cam_q,
            'publish_raw':      pub_raw,
            'camera_frame_id':  'camera_link',
        }],
        # No PYTHONPATH needed: pip install -e /path/to/Omega-Code/ handled it
    )

    # ---- capability detector -------------------------------------------
    capability_node = Node(
        package='omega_robot',
        executable='system_capabilities',
        name='omega_capability_detector',
        output='screen',
        parameters=[{'publish_interval': 5.0}],
        # No PYTHONPATH needed: pip install -e /path/to/Omega-Code/ handled it
    )

    # ---- static transforms (adjust XYZ/RPY to your physical robot) ------
    #
    # base_link -> camera_link
    #   camera is mounted 0.08 m forward, 0.10 m above base centre
    #   pitched 0 degrees (horizontal, looking forward)
    #   ROS optical convention: x=right, y=down, z=forward
    #   We add a -90 deg roll (-pi/2) and -90 deg yaw to align z-forward
    #   with the camera optical axis.  Adjust to your actual mount angle.
    #
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        output='screen',
        arguments=[
            # x      y      z      roll   pitch  yaw
            '0.08', '0.0', '0.10', '0.0', '0.0', '0.0',
            'base_link', 'camera_link',
        ],
    )

    # base_link -> ultrasonic_front
    #   ultrasonic sensor mounted 0.12 m forward, at base height
    static_tf_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_ultrasonic',
        output='screen',
        arguments=[
            '0.12', '0.0', '0.02', '0.0', '0.0', '0.0',
            'base_link', 'ultrasonic_front',
        ],
    )

    # ---- informational log ----------------------------------------------
    startup_log = LogInfo(msg=[
        'Omega Hybrid Launch starting\n'
        '  sim_mode=', sim, '\n'
        '  wheel_base=', wheel_base, 'm  wheel_radius=', wheel_radius, 'm\n'
        '  watchdog=', wd_timeout, 's  ramp=', ramp_type, '\n'
        '  camera=', launch_cam, ' (', cam_w, 'x', cam_h, ' @', cam_fps, 'fps)\n'
        '  PYTHONPATH prefix: ', project_root,
    ])

    # ---- assemble -------------------------------------------------------
    return LaunchDescription([
        *args,
        startup_log,
        global_sim,
        motor_node,
        sensor_node,
        camera_node,
        capability_node,
        static_tf_camera,
        static_tf_ultrasonic,
    ])
