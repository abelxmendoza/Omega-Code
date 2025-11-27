#!/bin/bash

set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Set Python unbuffered for better logging
export PYTHONUNBUFFERED=1

# Execute the command passed to the container
exec "$@"

