/*
# File: /Omega-Code/servers/robot-controller-backend/movement/movement.go
# Summary:
Handles WebSocket commands for the car's bi-directional movement (forward, backward, stop).
Also manages speed adjustments.
*/

package main

import (
	"encoding/json"
	"log"
	"net/http"

	"github.com/gorilla/websocket"
	"github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/gpio"
)

// MovementCommand represents a movement command received via WebSocket.
type MovementCommand struct {
	Command string `json:"command"` // "move-up", "move-down", "emergency-stop", "increase-speed", "decrease-speed"
	Speed   int    `json:"speed"`   // Speed percentage (0-100)
}

// WebSocket upgrader
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// SpeedController maintains the car's speed.
type SpeedController struct {
	speed int
}

// AdjustSpeed updates the car's speed.
func (sc *SpeedController) AdjustSpeed(delta int) {
	sc.speed += delta
	if sc.speed > 100 {
		sc.speed = 100
	} else if sc.speed < 0 {
		sc.speed = 0
	}
	log.Printf("Speed set to %d%%", sc.speed)
}

// Handle WebSocket commands for movement
func handleMovementCommands(ws *websocket.Conn, speedController *SpeedController) {
	for {
		var cmd MovementCommand
		err := ws.ReadJSON(&cmd)
		if err != nil {
			log.Printf("Error reading JSON: %v", err)
			break
		}

		log.Printf("Received command: %s, Speed: %d", cmd.Command, cmd.Speed)

		switch cmd.Command {
		case "move-up":
			log.Println("🚗 Moving forward")
			gpio.ActivateMotor("forward")
		case "move-down":
			log.Println("🚗 Moving backward")
			gpio.ActivateMotor("backward")
		case "increase-speed":
			speedController.AdjustSpeed(10)
		case "decrease-speed":
			speedController.AdjustSpeed(-10)
		case "emergency-stop":
			log.Println("🛑 Emergency Stop!")
			gpio.StopMotor()
		default:
			log.Printf("Unknown command: %s", cmd.Command)
		}
	}
}

// Start the movement WebSocket server
func main() {
	log.Println("🚀 Starting movement WebSocket server...")

	speedController := &SpeedController{speed: 0}

	http.HandleFunc("/movement", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("Upgrade error: %v", err)
			return
		}
		defer ws.Close()

		handleMovementCommands(ws, speedController)
	})

	addr := ":8081"
	log.Printf("Listening on %s...", addr)
	log.Fatal(http.ListenAndServe(addr, nil))
}
