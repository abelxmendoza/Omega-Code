FROM ros:humble

# Install essentials + CycloneDDS for inter-container communication
RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/omega_ws

# Copy ROS workspace source (ensure src/ directory exists)
COPY ros/src /root/omega_ws/src

# Create config directory and copy CycloneDDS configuration
RUN mkdir -p /root/omega_ws/config
COPY docker/ros2_robot/config/cyclonedds.xml /root/omega_ws/config/cyclonedds.xml

# Copy entrypoint script
COPY docker/ros2_robot/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Build ROS 2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

ENTRYPOINT ["/entrypoint.sh"]

