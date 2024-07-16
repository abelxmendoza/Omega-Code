package commands

import (
	"fmt"
	"log"
)

func ExecuteMotorCommand(cmd Command) {
	logCommand(cmd)

	var pin int
	var state string

	switch cmd.Command {
	case "move-up":
		pin = 19 // Example pin for moving the car forward
		state = "HIGH"
	case "move-down":
		pin = 20 // Example pin for moving the car backward
		state = "HIGH"
	case "move-left":
		pin = 21 // Example pin for moving the car left
		state = "HIGH"
	case "move-right":
		pin = 22 // Example pin for moving the car right
		state = "HIGH"
	default:
		log.Printf("Unknown motor command: %s\n", cmd.Command)
		return
	}

	err := executePythonScript("motor", fmt.Sprintf("%d", pin), state)
	if err != nil {
		log.Printf("Error executing Python script: %s\n", err)
	}
}
