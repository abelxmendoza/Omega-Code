// File: /Omega-Code/servers/robot_controller_backend/commands/servo_control.go

// Package commands handles the processing and execution of various commands for the robot controller.
package commands

import (
    "fmt"
    "log"

    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/common"
)

// ExecuteServoCommand executes a command to control a servo motor.
func ExecuteServoCommand(cmd Command) {
    logCommand(cmd)

    var channel, angle string

    switch cmd.Command {
    case "camera-left":
        channel = "horizontal"
        angle = fmt.Sprintf("%d", cmd.Angle) // Define specific angles or logic for left movement
    case "camera-right":
        channel = "horizontal"
        angle = fmt.Sprintf("%d", cmd.Angle) // Define specific angles or logic for right movement
    case "camera-up":
        channel = "vertical"
        angle = fmt.Sprintf("%d", cmd.Angle) // Define specific angles or logic for up movement
    case "camera-down":
        channel = "vertical"
        angle = fmt.Sprintf("%d", cmd.Angle) // Define specific angles or logic for down movement
    default:
        log.Printf("Unknown servo command: %s\n", cmd.Command)
        return
    }

    err := common.ExecutePythonScriptSimplified("servo", channel, angle)
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}
