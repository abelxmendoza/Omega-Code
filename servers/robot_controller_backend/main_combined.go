// File: /Omega-Code/servers/robot-controller-backend/main_combined.go

/*
This is the combined main entry point for starting the robot controller server with additional functionalities.
It initializes the server, starts the Python video server, and ROS nodes.
*/

package main

import (
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/commands"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/core"
)

func main() {
    go core.StartPythonVideoServer()
    go commands.StartROSNodes()
    core.StartServer()
}
