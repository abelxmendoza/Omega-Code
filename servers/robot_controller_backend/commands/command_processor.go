// File: /Omega-Code/servers/robot-controller-backend/commands/command_processor.go

// Package commands handles the processing and execution of various commands for the robot controller.
package commands

import (
    "bytes"
    "encoding/json"
    "fmt"
    "io"
    "log"
    "net/http"

    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/common"
)

// Command represents a command received from the client.
type Command struct {
    Command   string `json:"command"`
    Angle     int    `json:"angle,omitempty"`
    RequestID string `json:"request_id"`
}

// logCommand logs the details of a command.
func logCommand(cmd Command) {
    log.Printf("Executing command: %s with angle: %d and request_id: %s\n", cmd.Command, cmd.Angle, cmd.RequestID)
}

// HandleCommand handles incoming HTTP requests for executing commands.
func HandleCommand(w http.ResponseWriter, r *http.Request) {
    common.LogRequest(r)

    var body bytes.Buffer
    _, err := body.ReadFrom(r.Body)
    if err != nil {
        log.Printf("Error reading body: %s\n", err)
        http.Error(w, "Error reading body", http.StatusBadRequest)
        return
    }
    r.Body = io.NopCloser(&body)

    var cmd Command
    decoder := json.NewDecoder(r.Body)
    err = decoder.Decode(&cmd)
    if err != nil {
        log.Printf("Error decoding JSON: %s\n", err)
        http.Error(w, "Error decoding JSON", http.StatusBadRequest)
        return
    }

    log.Printf("Command received: %s", cmd.Command)

    switch cmd.Command {
    case "servo-horizontal", "servo-vertical":
        ExecuteServoCommand(cmd)
    case "move-up", "move-down", "move-left", "move-right":
        ExecuteMotorCommand(cmd)
    case "increase-speed", "decrease-speed", "buzz", "buzz-stop":
        log.Printf("Other command: %s", cmd.Command)
    default:
        log.Printf("Unknown command: %s\n", cmd.Command)
        http.Error(w, "Unknown command", http.StatusBadRequest)
    }

    log.Printf("Command executed: %s", cmd.Command)
    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}
