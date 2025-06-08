#!/bin/bash
# File: /Omega-Code/scripts/start_robot.sh

# This script initializes and starts various components for the Omega Robot project.
# It handles different hardware setups by starting the ROS core, launching ROS nodes 
# on the Raspberry Pi, starting the Go backend server, and launching the UI on the MacBook.

# Function to start ROS core
start_ros_core() {
    echo "Starting ROS core..."
    roscore &
    sleep 5  # Allow some time for the ROS core to initialize
}

# Function to launch ROS nodes on Raspberry Pi
start_raspberry_pi_nodes() {
    echo "Launching Raspberry Pi nodes..."
    ssh omega1@$TAILSCALE_IP_PI "source /opt/ros/noetic/setup.bash && roslaunch omega_robot robot_sensors.launch" &
}

# Function to launch ROS nodes on Jetson Nano
start_jetson_nano_nodes() {
    echo "Launching Jetson Nano nodes..."
    ssh omega1@$TAILSCALE_IP_NANO "source /opt/ros/noetic/setup.bash && roslaunch omega_robot robot_sensors.launch" &
}

# Function to start the UI on MacBook
start_ui() {
    echo "Starting the UI..."
    cd "$ROOT_DIR/ui/robot-controller-ui"
    npm start
}

# Function to start the Go backend server
start_go_backend() {
    echo "Starting Go backend server..."
    cd "$ROOT_DIR/servers/robot-controller-backend"
    go run main_combined.go &
}

# Load environment variables from .env file
# Determine project root. Allow override via OMEGA_CODE_ROOT
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Load environment variables from the backend .env file
ENV_FILE="${ENV_FILE:-$ROOT_DIR/servers/robot-controller-backend/.env}"
if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
else
    echo "Environment file not found: $ENV_FILE" >&2
    exit 1
fi

# Main script execution
main() {
    start_ros_core

    # Check the hardware setup and launch nodes accordingly
    if [ "$USE_RPI" = "true" ]; then
        start_raspberry_pi_nodes
    fi

    if [ "$USE_JETSON_NANO" = "true" ]; then
        start_jetson_nano_nodes
    fi

    start_go_backend
    start_ui
}

# Execute the main function
main
