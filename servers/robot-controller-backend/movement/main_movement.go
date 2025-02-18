/*
# File: /Omega-Code/servers/robot-controller-backend/main_movement.go
# Summary:
This file handles movement and speed control for the robot, including WebSocket communication for receiving commands. 
It supports directional movement (`move-up`, `move-down`, `move-left`, `move-right`), speed control (`increase-speed`, `decrease-speed`, `emergency-stop`), 
and buzzer activation (`buzz`, `buzz-stop`).
*/


package main

import (
	"encoding/json"
	"log"
	"net/http"

	"github.com/gorilla/websocket"
	"github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/commands"
)

// MovementCommand represents movement, speed, and buzzer commands.
type MovementCommand struct {
	Command   string `json:"command"`    // Command: movement, speed, or buzzer
	Speed     int    `json:"speed"`      // Optional: Speed adjustment
	RequestID string `json:"request_id"` // Unique request ID
}

// WebSocket upgrader for handling WebSocket connections
var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true // Allow all origins for testing; restrict in production.
	},
}

// SpeedController maintains the current speed state.
type SpeedController struct {
	speed int
}

// AdjustSpeed adjusts the current speed by a specified delta (positive or negative).
func (sc *SpeedController) AdjustSpeed(delta int) {
	sc.speed += delta
	if sc.speed > 100 {
		sc.speed = 100
	} else if sc.speed < 0 {
		sc.speed = 0
	}
}

// GetSpeed retrieves the current speed value.
func (sc *SpeedController) GetSpeed() int {
	return sc.speed
}

// SetSpeed sets the speed to a specific value.
func (sc *SpeedController) SetSpeed(value int) {
	if value >= 0 && value <= 100 {
		sc.speed = value
	}
}

// Handle WebSocket connections for movement commands
func handleMovementCommands(ws *websocket.Conn, speedController *SpeedController) {
	for {
		var cmd MovementCommand
		err := ws.ReadJSON(&cmd)
		if err != nil {
			log.Printf("Error reading JSON: %v", err)
			break
		}

		log.Printf("Received command: %s, Speed: %d", cmd.Command, cmd.Speed)

		// Execute commands based on the input
		switch cmd.Command {
		case "move-up":
			log.Println("Executing forward movement")
			commands.ExecuteMotorCommand(commands.Command{Command: "move-up"})
		case "move-down":
			log.Println("Executing backward movement")
			commands.ExecuteMotorCommand(commands.Command{Command: "move-down"})
		case "move-left":
			log.Println("Executing left movement")
			commands.ExecuteMotorCommand(commands.Command{Command: "move-left"})
		case "move-right":
			log.Println("Executing right movement")
			commands.ExecuteMotorCommand(commands.Command{Command: "move-right"})
		case "increase-speed":
			speedController.AdjustSpeed(10) // Increase speed by 10%
			log.Printf("Speed increased to %d%%", speedController.GetSpeed())
		case "decrease-speed":
			speedController.AdjustSpeed(-10) // Decrease speed by 10%
			log.Printf("Speed decreased to %d%%", speedController.GetSpeed())
		case "emergency-stop":
			speedController.SetSpeed(0) // Reset speed to 0%
			log.Println("Emergency stop triggered. Speed reset to 0%")
		case "buzz":
			log.Println("Activating buzzer")
			commands.ExecuteMotorCommand(commands.Command{Command: "buzz"})
		case "buzz-stop":
			log.Println("Stopping buzzer")
			commands.ExecuteMotorCommand(commands.Command{Command: "buzz-stop"})
		default:
			log.Printf("Unknown command: %s", cmd.Command)
		}
	}
}

// Start the movement WebSocket server
func main() {
	log.Println("Starting movement WebSocket server...")

	speedController := &SpeedController{speed: 0} // Initialize the speed controller

	http.HandleFunc("/movement", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("Upgrade error: %v", err)
			return
		}
		defer ws.Close()

		handleMovementCommands(ws, speedController)
	})

	addr := ":8081" // Port for the movement server
	log.Printf("Listening on %s...", addr)
	log.Fatal(http.ListenAndServe(addr, nil))
}
