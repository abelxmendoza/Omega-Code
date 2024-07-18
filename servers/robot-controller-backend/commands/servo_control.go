// File: /Omega-Code/servers/robot-controller-backend/commands/servo_control.go

package commands

import (
    "fmt"
    "log"
)

// Command struct to hold command details
type Command struct {
    Command   string
    Angle     int
    RequestID string
}

// ExecuteServoCommand executes a command to control a servo motor
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

    err := executePythonScript("servo", channel, angle)
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}

// executePythonScript runs a specified Python script with the given parameters
func executePythonScript(scriptType, param1, param2 string) error {
    cmdArgs := []string{fmt.Sprintf("%s_control.py", scriptType), param1, param2}
    command := execCommand("python3", cmdArgs...)
    var out bytes.Buffer
    var stderr bytes.Buffer
    command.Stdout = &out
    command.Stderr = &stderr
    err := command.Run()
    if err != nil {
        log.Printf("Error executing Python script: %s\n", stderr.String())
        return err
    }
    log.Printf("Python script output: %s\n", out.String())
    return nil
}

