// File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

/*
Lighting WebSocket Controller for LED Strips (NeoPixels)

File Location:
~/Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

This Go application provides a WebSocket server for real-time control of addressable RGB LED strips.
It receives JSON lighting commands from frontend clients, then launches a Python script (`led_control.py`)
to apply the requested color, mode, pattern, interval, and brightness using the rpi_ws281x library.

Updates in this version:
- Adds WS heartbeat support: responds to {"type":"ping","ts"} with {"type":"pong","ts"} for UI latency/health.
- Fixes brightness defaulting: brightness is now optional (*float64); default to 1.0 only when omitted (nil).
  Explicit 0.0 is respected (e.g., to turn LEDs off) and all values are clamped to [0,1].
- Minor: uses WriteJSON for the welcome envelope.
- Safety: runs the Python effect command with a short timeout to avoid hanging processes.

Features:
- Handles WebSocket connections for real-time control.
- Receives lighting commands as JSON (color, mode, pattern, interval, brightness).
- Executes Python scripts to drive LED effects.

Expected JSON payload from frontend (supports both int and hex string):
{
    "color": "#ff0000",        // Hex string (recommended, e.g., "#ff0000" for red)
    "mode": "single",          // e.g., "single", "multi", "rainbow"
    "pattern": "static",       // e.g., "static", "blink", "fade", "off"
    "interval": 500,           // For animation speed, in ms
    "brightness": 0.85         // Float [0,1], optional; omitted => 1.0; 0.0 is allowed
}
or
{
    "color": 16711680,         // 24-bit RGB int (e.g., 0xFF0000 for red)
    ...
}

Heartbeat example:
Client → {"type":"ping","ts": 1723220000000}
Server ← {"type":"pong","ts": 1723220000000}
*/

package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os/exec"
	"regexp"
	"strings"
	"time"

	"github.com/gorilla/websocket"
)

// LightingCommand supports both string and int color fields for compatibility.
// Brightness is a *float64 so we can tell if it was omitted (nil) vs provided (including 0).
type LightingCommand struct {
	Color      interface{} `json:"color"`                // Can be hex string or int (e.g., "#ff0000" or 16711680)
	Mode       string      `json:"mode"`                 // e.g., "single", "multi", "rainbow"
	Pattern    string      `json:"pattern"`              // e.g., "static", "blink", "fade", "off"
	Interval   int         `json:"interval"`             // For dynamic patterns, in milliseconds
	Brightness *float64    `json:"brightness,omitempty"` // Optional, 0.0–1.0 (nil => default 1.0)
}

// WebSocket upgrader with permissive CORS
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// hexColorString converts any supported color input to a 6-digit hex string
func hexColorString(color interface{}) (string, error) {
	switch c := color.(type) {
	case float64: // JSON numbers come in as float64
		return fmt.Sprintf("%06x", int(c)), nil
	case string:
		hex := strings.TrimPrefix(strings.TrimPrefix(c, "#"), "0x")
		// Validate it's 6 hex digits
		if len(hex) != 6 || !regexp.MustCompile(`^[0-9a-fA-F]{6}$`).MatchString(hex) {
			return "", fmt.Errorf("invalid hex color: %s", c)
		}
		return strings.ToLower(hex), nil
	default:
		return "", fmt.Errorf("unsupported color type: %T", c)
	}
}

// clampFloat limits v to [min, max]
func clampFloat(v, min, max float64) float64 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

// handleLighting manages a single WebSocket connection for LED control
func handleLighting(ws *websocket.Conn) {
	defer func() {
		log.Println("[DISCONNECTED] Lighting client connection closed")
		ws.Close()
	}()
	log.Println("Lighting WebSocket connection established")

	// Send connection confirmation to the frontend
	_ = ws.WriteJSON(map[string]interface{}{
		"status":  "connected",
		"service": "lighting",
		"message": "Lighting WebSocket connection established",
		"ts":      time.Now().UnixMilli(),
	})

	for {
		_, msg, err := ws.ReadMessage()
		if err != nil {
			log.Printf("WebSocket read error: %v", err)
			break
		}

		// Peek for heartbeat pings without assuming the payload is a LightingCommand
		var envelope map[string]json.RawMessage
		if err := json.Unmarshal(msg, &envelope); err == nil {
			if tRaw, ok := envelope["type"]; ok {
				var typ string
				_ = json.Unmarshal(tRaw, &typ)
				if typ == "ping" {
					var ts any
					_ = json.Unmarshal(envelope["ts"], &ts) // echo whatever came in
					_ = ws.WriteJSON(map[string]interface{}{"type": "pong", "ts": ts})
					continue // Skip normal command handling
				}
			}
		}

		// Handle lighting command
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

		// Brightness: default to 1.0 only when omitted; otherwise clamp to [0,1]
		brightness := 1.0
		if command.Brightness != nil {
			brightness = clampFloat(*command.Brightness, 0.0, 1.0)
		}

		// Interval: guard against negatives
		if command.Interval < 0 {
			command.Interval = 0
		}

		// Build the command for the Python script (includes brightness as the 5th argument)
		pythonCmd := fmt.Sprintf(
			"python3 led_control.py %s %s %s %d %.3f",
			hexColor, command.Mode, command.Pattern, command.Interval, brightness,
		)
		log.Printf("Executing: %s", pythonCmd)

		// Run the Python script to control the LEDs with a short timeout
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		cmd := exec.CommandContext(ctx, "bash", "-c", pythonCmd)
		output, err := cmd.CombinedOutput()
		cancel()

		if ctx.Err() == context.DeadlineExceeded {
			log.Printf("Command timed out after 5s")
		}
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
