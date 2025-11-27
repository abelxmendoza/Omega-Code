FROM ros:humble

RUN apt update && apt install -y \
    python3-pip python3-opencv python3-numpy \
    python3-picamera2 libcamera-apps libatlas-base-dev \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-sensor-msgs ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    git vim nano curl wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy opencv-python

# Initialize rosdep
RUN rosdep init || true

WORKDIR /workspace

