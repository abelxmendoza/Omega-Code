// File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

/*
Lighting WebSocket Controller for LED Strips (NeoPixels)

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

This Go application provides a WebSocket server for real-time control of addressable RGB LED strips.
It receives JSON lighting commands from frontend clients, then launches a Python script (`led_control.py`)
to apply the requested color, mode, pattern, interval, and brightness using the rpi_ws281x library.

Features:
- Handles WebSocket connections for real-time control.
- Receives lighting commands as JSON (color, mode, pattern, interval, brightness).
- Executes Python scripts to drive LED effects.

Expected JSON payload from frontend (supports both int and hex string):
{
    "color": "#ff0000",        // Hex string (recommended, e.g. "#ff0000" for red)
    "mode": "single",          // e.g., "single", "multi"
    "pattern": "static",       // e.g., "static", "blink"
    "interval": 500,           // For animation speed, in ms
    "brightness": 0.85         // Float [0,1], optional (default: 1.0)
}
or
{
    "color": 16711680,         // 24-bit RGB int (e.g., 0xFF0000 for red)
    ...
}
*/

package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os/exec"
	"regexp"
	"strings"

	"github.com/gorilla/websocket"
)

// LightingCommand supports both string and int color fields for compatibility
type LightingCommand struct {
	Color      interface{} `json:"color"`      // Can be hex string or int (e.g., "#ff0000" or 16711680)
	Mode       string      `json:"mode"`       // e.g., "single", "multi", "rainbow"
	Pattern    string      `json:"pattern"`    // e.g., "static", "blink", "fade"
	Interval   int         `json:"interval"`   // For dynamic patterns, in milliseconds
	Brightness float64     `json:"brightness"` // Optional, 0.0–1.0 (default: 1.0)
}

// WebSocket upgrader with permissive CORS
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// hexColorString converts any supported color input to a 6-digit hex string
func hexColorString(color interface{}) (string, error) {
	switch c := color.(type) {
	case float64:
		// Received as a number (int in JSON comes as float64)
		return fmt.Sprintf("%06x", int(c)), nil
	case string:
		hex := c
		hex = strings.TrimPrefix(hex, "#")
		hex = strings.TrimPrefix(hex, "0x")
		// Validate it's 6 hex digits
		if len(hex) != 6 || !regexp.MustCompile(`^[0-9a-fA-F]{6}$`).MatchString(hex) {
			return "", fmt.Errorf("invalid hex color: %s", c)
		}
		return strings.ToLower(hex), nil
	default:
		return "", fmt.Errorf("unsupported color type: %T", c)
	}
}

// handleLighting manages a single WebSocket connection for LED control
func handleLighting(ws *websocket.Conn) {
	defer func() {
		log.Println("[DISCONNECTED] Lighting client connection closed")
		ws.Close()
	}()
	log.Println("Lighting WebSocket connection established")

	// Send connection confirmation to the frontend
	welcomeMsg := map[string]string{
		"status":  "connected",
		"service": "lighting",
		"message": "Lighting WebSocket connection established",
	}
	welcomeBytes, _ := json.Marshal(welcomeMsg)
	ws.WriteMessage(websocket.TextMessage, welcomeBytes)

	for {
		_, msg, err := ws.ReadMessage()
		if err != nil {
			log.Printf("WebSocket read error: %v", err)
			break
		}

		var command LightingCommand
		if err := json.Unmarshal(msg, &command); err != nil {
			log.Printf("JSON unmarshal error: %v", err)
			continue
		}
		log.Printf("Received Lighting Command: %+v", command)

		hexColor, err := hexColorString(command.Color)
		if err != nil {
			log.Printf("Color parsing error: %v", err)
			continue
		}

		brightness := command.Brightness
		if brightness < 0.0 || brightness > 1.0 {
			log.Printf("Invalid brightness value: %v (should be 0.0–1.0); using 1.0", brightness)
			brightness = 1.0
		}
		// Default to 1.0 if zero (assuming omitted)
		if brightness == 0 {
			brightness = 1.0
		}

		// Build the command for the Python script (now includes brightness as the 5th argument)
		pythonCmd := fmt.Sprintf(
			"python3 led_control.py %s %s %s %d %.3f",
			hexColor, command.Mode, command.Pattern, command.Interval, brightness,
		)
		log.Printf("Executing: %s", pythonCmd)

		// Run the Python script to control the LEDs
		cmd := exec.Command("bash", "-c", pythonCmd)
		output, err := cmd.CombinedOutput()
		if err != nil {
			log.Printf("Command execution failed: %v", err)
			log.Printf("Output: %s", string(output))
		} else {
			log.Printf("Command output: %s", string(output))
		}
	}
}

// main function starts the lighting WebSocket server on port 8082
func main() {
	http.HandleFunc("/lighting", func(w http.ResponseWriter, r *http.Request) {
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("WebSocket upgrade failed: %v", err)
			return
		}
		log.Printf("[CONNECTED] Lighting client: %s", r.RemoteAddr)
		handleLighting(conn)
	})

	log.Println("Lighting WebSocket server is running on port 8082")
	log.Fatal(http.ListenAndServe(":8082", nil))
}
