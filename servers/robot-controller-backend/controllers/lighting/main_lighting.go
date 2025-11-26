// File: /Omega-Code/servers/robot-controller-backend/controllers/lighting/main_lighting.go

/*
Lighting WebSocket Controller for LED Strips (NeoPixels)

This Go application provides a WebSocket server for real-time control of addressable RGB LED strips.
It receives JSON lighting commands from frontend clients, then launches a privileged wrapper script
(run_led.sh → sudo + venv python led_control.py) to apply color/mode/pattern/interval/brightness.

Key points in this version:
- Uses a wrapper (RUN_LED) so the Go process itself does not need sudo.
- WS heartbeat supported: {"type":"ping","ts"} → {"type":"pong","ts"}.
- Brightness is optional (*float64); default to 1.0 only when omitted (nil). 0.0 allowed and respected.
- Short timeout on each invocation to avoid hanging processes.
*/

package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/exec"
	"regexp"
	"strconv"
	"strings"
	"time"

	"github.com/gorilla/websocket"
)

// getRunLedPath returns the path to the privileged wrapper script.
// Can be overridden via RUN_LED_PATH environment variable.
func getRunLedPath() string {
	if path := os.Getenv("RUN_LED_PATH"); path != "" {
		return path
	}
	// Default: assume script is in same directory as this Go binary
	// Try to resolve relative to current working directory
	if wd, err := os.Getwd(); err == nil {
		scriptPath := wd + "/run_led.sh"
		if _, err := os.Stat(scriptPath); err == nil {
			return scriptPath
		}
	}
	// Fallback to absolute path (for Raspberry Pi)
	return "/home/omega1/Omega-Code/servers/robot-controller-backend/controllers/lighting/run_led.sh"
}

// LightingCommand supports both string and int color fields for compatibility.
// Brightness is a *float64 so we can tell if it was omitted (nil) vs provided (including 0).
type LightingCommand struct {
	Color      interface{} `json:"color"`                // Hex string "#rrggbb" or 24-bit int
	Mode       string      `json:"mode"`                 // "single", "multi", "rainbow", etc.
	Pattern    string      `json:"pattern"`              // "static", "blink", "fade", "off", etc.
	Interval   int         `json:"interval"`             // ms (animation speed)
	Brightness *float64    `json:"brightness,omitempty"` // [0,1]; nil => default 1.0
}

// WebSocket upgrader with permissive CORS
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// hexColorString converts any supported color input to a 6-digit hex string (no leading '#').
func hexColorString(color interface{}) (string, error) {
	switch c := color.(type) {
	case float64: // JSON numbers arrive as float64
		return fmt.Sprintf("%06x", int(c)), nil
	case string:
		hex := strings.TrimPrefix(strings.TrimPrefix(c, "#"), "0x")
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

		// Build an arg list for the wrapper (no shell needed)
		args := []string{
			hexColor,
			command.Mode,
			command.Pattern,
			strconv.Itoa(command.Interval),
			fmt.Sprintf("%.3f", brightness),
		}

		runLedPath := getRunLedPath()
		log.Printf("Executing: %s %v", runLedPath, args)

		// Run the wrapper with a short timeout
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		cmd := exec.CommandContext(ctx, runLedPath, args...)
		output, err := cmd.CombinedOutput()
		cancel()

		if ctx.Err() == context.DeadlineExceeded {
			log.Printf("Command timed out after 5s")
		}
		if err != nil {
			log.Printf("Command execution failed: %v", err)
			log.Printf("Output: %s", string(output))
			// Optionally notify client of failure
			_ = ws.WriteJSON(map[string]any{
				"type":   "lighting_result",
				"ok":     false,
				"error":  err.Error(),
				"output": string(output),
				"ts":     time.Now().UnixMilli(),
			})
		} else {
			log.Printf("Command output: %s", string(output))
			_ = ws.WriteJSON(map[string]any{
				"type":   "lighting_result",
				"ok":     true,
				"output": string(output),
				"ts":     time.Now().UnixMilli(),
			})
		}
	}
}

// main function starts the lighting WebSocket server
func main() {
	// Get configuration from environment variables
	port := os.Getenv("PORT_LIGHTING")
	if port == "" {
		port = "8082"
	}
	path := os.Getenv("LIGHTING_PATH")
	if path == "" {
		path = "/lighting"
	}

	// Verify run_led.sh exists
	runLedPath := getRunLedPath()
	if _, err := os.Stat(runLedPath); os.IsNotExist(err) {
		log.Fatalf("ERROR: run_led.sh not found at %s. Set RUN_LED_PATH environment variable or ensure run_led.sh exists.", runLedPath)
	}

	http.HandleFunc(path, func(w http.ResponseWriter, r *http.Request) {
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("WebSocket upgrade failed: %v", err)
			return
		}
		log.Printf("[CONNECTED] Lighting client: %s", r.RemoteAddr)
		handleLighting(conn)
	})

	log.Printf("Lighting WebSocket server is running on port %s at path %s", port, path)
	log.Printf("Using LED script: %s", runLedPath)
	log.Fatal(http.ListenAndServe(":"+port, nil))
}
