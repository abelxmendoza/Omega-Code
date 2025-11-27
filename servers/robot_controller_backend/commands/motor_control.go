/*
# File: /Omega-Code/servers/robot-controller-backend/commands/motor_control.go
# Summary:
This file provides functionality for motor and buzzer control. Includes support for horn-like buzzer behavior.
*/

package commands

import (
    "fmt"
    "log"

    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/common"
)

// ExecuteMotorCommand executes a motor or buzzer control command.
func ExecuteMotorCommand(cmd Command) {
    logCommand(cmd)

    var scriptType string
    var param1 string

    switch cmd.Command {
    case "move-up":
        scriptType = "motor"
        param1 = "19" // Pin for moving forward
    case "move-down":
        scriptType = "motor"
        param1 = "20" // Pin for moving backward
    case "move-left":
        scriptType = "motor"
        param1 = "21" // Pin for moving left
    case "move-right":
        scriptType = "motor"
        param1 = "22" // Pin for moving right
    case "buzz":
        scriptType = "buzzer"
        param1 = "on"
    case "buzz-stop":
        scriptType = "buzzer"
        param1 = "off"
    default:
        log.Printf("Unknown command: %s\n", cmd.Command)
        return
    }

    err := common.ExecutePythonScriptSimplified(scriptType, param1)
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}
