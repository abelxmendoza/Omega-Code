import sys
import os

# Add ros/ so 'from scripts.xxx import ...' works
_ros_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _ros_dir not in sys.path:
    sys.path.insert(0, _ros_dir)

# Add ros/scripts/ so bare 'import d_star_lite' style imports work
_scripts_dir = os.path.join(_ros_dir, "scripts")
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

# Make ROS2 Python packages importable without sourcing setup.bash
_ros2_py = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ros2_py) and _ros2_py not in sys.path:
    sys.path.insert(0, _ros2_py)
