// File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

/*
Lighting Controller

This Go application provides a WebSocket server for controlling an LED strip.
It processes lighting commands (color, mode, pattern, interval) sent from clients and executes them via Python scripts.

Features:
- Handles WebSocket connections for real-time control.
- Executes Python scripts to configure LED behavior.
*/

package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os/exec"

	"github.com/gorilla/websocket"
)

// LightingCommand represents the structure of a lighting command
type LightingCommand struct {
	Color    int    `json:"color"`    // 24-bit RGB color value
	Mode     string `json:"mode"`     // Lighting mode (e.g., single, multi)
	Pattern  string `json:"pattern"`  // Lighting pattern (e.g., static, blink)
	Interval int    `json:"interval"` // Interval for dynamic patterns
}

// WebSocket upgrader configuration
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true // Allow all origins for simplicity
	},
}

// handleLighting processes incoming WebSocket messages for lighting control
func handleLighting(ws *websocket.Conn) {
	defer ws.Close()
	log.Println("Lighting WebSocket connection established")

	for {
		var command LightingCommand
		err := ws.ReadJSON(&command)
		if err != nil {
			log.Printf("Error reading WebSocket message: %v\n", err)
			break
		}

		log.Printf("Received Command: %+v\n", command)

		// Build the Python script command
		pythonCmd := fmt.Sprintf("python3 led_control.py %06x %s %s %d",
			command.Color, command.Mode, command.Pattern, command.Interval)

		// Execute the Python script
		log.Printf("Executing: %s\n", pythonCmd)
		cmd := exec.Command("bash", "-c", pythonCmd)
		output, err := cmd.CombinedOutput()
		if err != nil {
			log.Printf("Command execution failed: %v\n", err)
		} else {
			log.Printf("Command output: %s\n", string(output))
		}
	}
}

func main() {
	http.HandleFunc("/lighting", func(w http.ResponseWriter, r *http.Request) {
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("WebSocket upgrade failed: %v\n", err)
			return
		}
		handleLighting(conn)
	})

	log.Println("Lighting WebSocket server is running on port 8082")
	log.Fatal(http.ListenAndServe(":8082", nil))
}
