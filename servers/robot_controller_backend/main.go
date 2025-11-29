// File: /Omega-Code/servers/robot_controller_backend/main.go

/*
This is the main entry point for starting the robot controller server.
It initializes the server and handles WebSocket connections for various robot control commands.
*/

package main

import (
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/core"
)

func main() {
    core.StartServer()
}
