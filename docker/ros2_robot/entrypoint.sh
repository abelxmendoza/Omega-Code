#!/bin/bash

set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace setup
source /root/omega_ws/install/setup.bash

# Set CycloneDDS environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export CYCLONEDDS_URI=file:///root/omega_ws/config/cyclonedds.xml

# Execute the command passed to the container
exec "$@"

