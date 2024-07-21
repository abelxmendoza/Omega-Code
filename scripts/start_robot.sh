#!/bin/bash
# File: /Omega-Code/scripts/start_robot.sh

# This script initializes and starts various components for the Omega Robot project.
# It starts the ROS core, launches ROS nodes on the Raspberry Pi, starts the Go backend server,
# and launches the UI on the MacBook.

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

# Function to start the UI on MacBook
start_ui() {
    echo "Starting the UI..."
    cd /Users/abel_elreaper/Desktop/Omega-Code/ui/robot-controller-ui
    npm start
}

# Function to start the Go backend server
start_go_backend() {
    echo "Starting Go backend server..."
    cd /Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend
    go run main_combined.go &
}

# Load environment variables from .env file
source /Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend/.env

# Main script execution
main() {
    start_ros_core
    start_raspberry_pi_nodes
    start_go_backend
    start_ui
}

# Execute the main function
main
