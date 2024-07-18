package commands

import (
	"fmt"
	"log"
)

func ExecuteServoCommand(cmd Command) {
	logCommand(cmd)

	var channel, angle string

	switch cmd.Command {
	case "camera-left":
		channel = "0"
		angle = "left" // Define specific angles or logic for left movement
	case "camera-right":
		channel = "0"
		angle = "right" // Define specific angles or logic for right movement
	case "camera-up":
		channel = "1"
		angle = "up" // Define specific angles or logic for up movement
	case "camera-down":
		channel = "1"
		angle = "down" // Define specific angles or logic for down movement
	default:
		log.Printf("Unknown servo command: %s\n", cmd.Command)
		return
	}

	err := executePythonScript("servo", channel, angle, "", "")
	if err != nil {
		log.Printf("Error executing Python script: %s\n", err)
	}
}
