package commands

import (
    "log"
    "os/exec"
)

// StartROSNodes starts the ROS nodes specified in the launch file.
func StartROSNodes() {
    cmd := exec.Command("roslaunch", "your_ros_package_name", "robot_sensors.launch")
    cmd.Stdout = os.Stdout
    cmd.Stderr = os.Stderr
    err := cmd.Start()
    if err != nil {
        log.Fatalf("Error starting ROS nodes: %s", err)
    }
    log.Printf("Started ROS nodes with PID %d", cmd.Process.Pid)
}
