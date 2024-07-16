package commands

import (
	"log"
	"fmt"
)

func ExecuteServoCommand(cmd Command) {
    logCommand(cmd)

    var channel, angle string

    switch cmd.Command {
    case "servo-horizontal":
        channel = "0"
        angle = fmt.Sprintf("%d", cmd.Angle)
    case "servo-vertical":
        channel = "1"
        angle = fmt.Sprintf("%d", cmd.Angle)
    default:
        log.Printf("Unknown servo command: %s\n", cmd.Command)
        return
    }

    err := executePythonScript("servo", channel, angle)
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}
