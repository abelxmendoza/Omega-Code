// File: /Omega-Code/servers/robot_controller_backend/controllers/lighting/main_lighting.go

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
	"net"
	"net/http"
	"os"
	"os/exec"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// Pre-compiled regex for hex color validation (cached for performance)
var hexColorRegex = regexp.MustCompile(`^[0-9a-fA-F]{6}$`)

// Response pool for reducing allocations
var responsePool = sync.Pool{
	New: func() interface{} {
		return make(map[string]interface{}, 4)
	},
}

// Pong response pool (reused for ping/pong)
var pongPool = sync.Pool{
	New: func() interface{} {
		return make(map[string]interface{}, 2)
	},
}

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
	return "/home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting/run_led.sh"
}

// LightingCommand supports both string and int color fields for compatibility.
// Brightness is a *float64 so we can tell if it was omitted (nil) vs provided (including 0).
type LightingCommand struct {
	Color      interface{} `json:"color"`                 // Hex string "#rrggbb" or 24-bit int
	Color2     interface{} `json:"color2,omitempty"`      // Hex string "#rrggbb" or 24-bit int (for dual mode)
	Mode       string      `json:"mode"`                  // "single", "dual", "rainbow", etc.
	Pattern    string      `json:"pattern"`               // "static", "blink", "fade", "off", etc.
	Interval   int         `json:"interval"`              // ms (animation speed)
	Brightness *float64    `json:"brightness,omitempty"`  // [0,1]; nil => default 1.0
}

// WebSocket upgrader with permissive CORS
var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// hexColorString converts any supported color input to a 6-digit hex string (no leading '#').
// Optimized with cached regex and efficient string operations.
func hexColorString(color interface{}) (string, error) {
	switch c := color.(type) {
	case float64: // JSON numbers arrive as float64
		// Validate range before conversion
		if c < 0 || c > 0xFFFFFF {
			return "", fmt.Errorf("color value out of range: %.0f (must be 0-16777215)", c)
		}
		return fmt.Sprintf("%06x", int(c)), nil
	case string:
		// Optimized: single pass trim and validation
		hex := c
		if len(hex) > 0 && hex[0] == '#' {
			hex = hex[1:]
		} else if len(hex) > 1 && hex[0:2] == "0x" {
			hex = hex[2:]
		}
		
		if len(hex) != 6 {
			return "", fmt.Errorf("invalid hex color length: %s (expected 6 hex digits, got %d)", c, len(hex))
		}
		
		if !hexColorRegex.MatchString(hex) {
			return "", fmt.Errorf("invalid hex color format: %s (must be 0-9, a-f, A-F)", c)
		}
		
		return strings.ToLower(hex), nil
	case int:
		if c < 0 || c > 0xFFFFFF {
			return "", fmt.Errorf("color value out of range: %d (must be 0-16777215)", c)
		}
		return fmt.Sprintf("%06x", c), nil
	default:
		return "", fmt.Errorf("unsupported color type: %T (expected string, int, or float64)", c)
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
		if r := recover(); r != nil {
			log.Printf("❌ [PANIC] Lighting handler panic: %v", r)
		}
		log.Println("🔌 [DISCONNECTED] Lighting client connection closed")
		ws.Close()
	}()
	
	clientAddr := ws.RemoteAddr().String()
	log.Printf("✅ [CONNECTED] Lighting WebSocket client connected: %s", clientAddr)
	log.Printf("   📡 Connection established at %s", time.Now().Format("2006-01-02 15:04:05"))

	// Pre-allocate welcome message (reused from pool)
	welcomeMsg := responsePool.Get().(map[string]interface{})
	welcomeMsg["status"] = "connected"
	welcomeMsg["service"] = "lighting"
	welcomeMsg["message"] = "Lighting WebSocket connection established"
	welcomeMsg["ts"] = time.Now().UnixMilli()
	
	if err := ws.WriteJSON(welcomeMsg); err != nil {
		log.Printf("❌ [ERROR] Failed to send welcome message to %s: %v", clientAddr, err)
		responsePool.Put(welcomeMsg)
		return
	}
	responsePool.Put(welcomeMsg)

	for {
		log.Printf("   📡 [READ] Waiting for message from %s...", clientAddr)
		_, msg, err := ws.ReadMessage()
		if err != nil {
			log.Printf("   ❌ [READ] WebSocket read error from %s: %v", clientAddr, err)
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("   ⚠️  [READ] Unexpected close error: %v", err)
			}
			break
		}
		
		log.Printf("   📨 [READ] Received %d bytes from %s", len(msg), clientAddr)
		log.Printf("   📄 [READ] Raw message: %s", string(msg))

		// Optimized heartbeat detection: fast path for ping/pong
		// Check for ping without full unmarshal if possible
		if len(msg) > 8 && msg[0] == '{' {
			var envelope map[string]json.RawMessage
			if err := json.Unmarshal(msg, &envelope); err == nil {
				if tRaw, ok := envelope["type"]; ok {
					var typ string
					if err := json.Unmarshal(tRaw, &typ); err == nil && typ == "ping" {
						// Reuse pong response from pool
						pongMsg := pongPool.Get().(map[string]interface{})
						pongMsg["type"] = "pong"
						var ts any
						if tsRaw, ok := envelope["ts"]; ok {
							_ = json.Unmarshal(tsRaw, &ts)
							pongMsg["ts"] = ts
						} else {
							pongMsg["ts"] = time.Now().UnixMilli()
						}
						if err := ws.WriteJSON(pongMsg); err != nil {
							log.Printf("⚠️ [WARN] Failed to send pong to %s: %v", clientAddr, err)
						}
						pongPool.Put(pongMsg)
						continue // Skip normal command handling
					}
				}
			}
		}

		// Handle lighting command with comprehensive error handling
		var command LightingCommand
		if err := json.Unmarshal(msg, &command); err != nil {
			log.Printf("❌ [ERROR] JSON unmarshal failed for %s: %v | Raw: %s", clientAddr, err, string(msg))
			// Send error response to client
			errorResp := responsePool.Get().(map[string]interface{})
			errorResp["type"] = "error"
			errorResp["error"] = fmt.Sprintf("Invalid JSON: %v", err)
			errorResp["ts"] = time.Now().UnixMilli()
			_ = ws.WriteJSON(errorResp)
			responsePool.Put(errorResp)
			continue
		}
		
		log.Printf("📨 [CMD] Received lighting command from %s", clientAddr)
		log.Printf("   🎨 Color: %v | Mode: %s | Pattern: %s | Interval: %dms | Brightness: %v",
			command.Color, command.Mode, command.Pattern, command.Interval, command.Brightness)

		log.Printf("   🔄 [PROCESS] Parsing color...")
		hexColor, err := hexColorString(command.Color)
		if err != nil {
			log.Printf("❌ [ERROR] Color parsing FAILED for %s", clientAddr)
			log.Printf("   Error: %v", err)
			errorResp := responsePool.Get().(map[string]interface{})
			errorResp["type"] = "error"
			errorResp["error"] = fmt.Sprintf("Color parsing error: %v", err)
			errorResp["ts"] = time.Now().UnixMilli()
			_ = ws.WriteJSON(errorResp)
			responsePool.Put(errorResp)
			continue
		}
		log.Printf("   ✅ [PROCESS] Color parsed: #%s", hexColor)

		// Parse color2 if provided (for dual mode)
		hexColor2 := "000000" // Default to black if not provided
		if command.Color2 != nil {
			hexColor2Parsed, err2 := hexColorString(command.Color2)
			if err2 != nil {
				log.Printf("⚠️ [WARN] Color2 parsing failed, using default black: %v", err2)
			} else {
				hexColor2 = hexColor2Parsed
				log.Printf("   ✅ [PROCESS] Color2 parsed: #%s", hexColor2)
			}
		}

		// Brightness: default to 1.0 only when omitted; otherwise clamp to [0,1]
		brightness := 1.0
		if command.Brightness != nil {
			brightness = clampFloat(*command.Brightness, 0.0, 1.0)
			if brightness != *command.Brightness {
				log.Printf("⚠️ [WARN] Brightness clamped from %.3f to %.3f", *command.Brightness, brightness)
			}
		}

		// Interval: guard against negatives and set reasonable maximum
		if command.Interval < 0 {
			log.Printf("⚠️ [WARN] Negative interval %dms corrected to 0ms", command.Interval)
			command.Interval = 0
		} else if command.Interval > 60000 {
			log.Printf("⚠️ [WARN] Interval %dms exceeds maximum (60000ms), clamping", command.Interval)
			command.Interval = 60000
		}

		// Build an arg list for the wrapper (no shell needed)
		// Format: <hexColor> <hexColor2> <mode> <pattern> <interval> <brightness>
		args := []string{
			hexColor,
			hexColor2,
			command.Mode,
			command.Pattern,
			strconv.Itoa(command.Interval),
			fmt.Sprintf("%.3f", brightness),
		}

		runLedPath := getRunLedPath()
		log.Printf("   ⚡ [EXEC] Executing LED command...")
		log.Printf("      Script: %s", runLedPath)
		log.Printf("      Args: %v", args)

		// Run the wrapper with timeout and proper error handling
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		
		startTime := time.Now()
		log.Printf("   ⏳ [EXEC] Command started at %s", startTime.Format("15:04:05.000"))
		cmd := exec.CommandContext(ctx, runLedPath, args...)
		output, err := cmd.CombinedOutput()
		duration := time.Since(startTime)
		log.Printf("   ⏱️  [EXEC] Command completed in %v", duration)

		// Reuse response map from pool
		resultResp := responsePool.Get().(map[string]interface{})
		resultResp["type"] = "lighting_result"
		resultResp["ts"] = time.Now().UnixMilli()

		if ctx.Err() == context.DeadlineExceeded {
			log.Printf("⏱️  [TIMEOUT] LED command EXCEEDED 5s timeout for %s", clientAddr)
			log.Printf("   ⚠️  Command took longer than 5 seconds and was cancelled")
			resultResp["ok"] = false
			resultResp["error"] = "Command execution timeout (exceeded 5s)"
			resultResp["output"] = string(output)
			_ = ws.WriteJSON(resultResp)
			responsePool.Put(resultResp)
			continue
		}
		
		if err != nil {
			log.Printf("❌ [ERROR] LED command FAILED for %s", clientAddr)
			log.Printf("   Duration: %v | Error: %v", duration, err)
			log.Printf("   Output: %s", string(output))
			log.Printf("   💡 Check LED script permissions and hardware connections")
			resultResp["ok"] = false
			resultResp["error"] = err.Error()
			resultResp["output"] = string(output)
			_ = ws.WriteJSON(resultResp)
			responsePool.Put(resultResp)
			continue
		}
		
		log.Printf("✅ [SUCCESS] LED command completed successfully for %s", clientAddr)
		log.Printf("   ⏱️  Duration: %v", duration)
		if len(output) > 0 {
			log.Printf("   📤 Output: %s", strings.TrimSpace(string(output)))
		}
		log.Printf("   📡 Sending success response to client...")
		resultResp["ok"] = true
		resultResp["output"] = strings.TrimSpace(string(output))
		resultResp["duration_ms"] = duration.Milliseconds()
		_ = ws.WriteJSON(resultResp)
		responsePool.Put(resultResp)
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

	// Verify run_led.sh exists with detailed error message
	runLedPath := getRunLedPath()
	if stat, err := os.Stat(runLedPath); os.IsNotExist(err) {
		log.Fatalf("❌ [FATAL] run_led.sh not found at %s\n"+
			"   Solution: Set RUN_LED_PATH environment variable or ensure run_led.sh exists in:\n"+
			"   - Current directory: %s\n"+
			"   - Default path: /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting/run_led.sh",
			runLedPath, getRunLedPath())
	} else if stat.Mode().Perm()&0111 == 0 {
		log.Fatalf("❌ [FATAL] run_led.sh exists but is not executable: %s\n"+
			"   Solution: chmod +x %s", runLedPath, runLedPath)
	} else {
		log.Printf("✅ [INIT] Verified LED script: %s (executable)", runLedPath)
	}

	// Health check endpoint for connectivity testing
	http.HandleFunc("/health", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		json.NewEncoder(w).Encode(map[string]interface{}{
			"status":    "ok",
			"service":   "lighting",
			"port":      port,
			"path":      path,
			"timestamp": time.Now().Unix(),
		})
	})

	http.HandleFunc(path, func(w http.ResponseWriter, r *http.Request) {
		clientIP := r.RemoteAddr
		log.Printf("📥 [REQUEST] Incoming connection attempt from %s", clientIP)
		log.Printf("   Method: %s | Path: %s | Protocol: %s", r.Method, r.URL.Path, r.Proto)
		log.Printf("   Headers - Upgrade: %s | Connection: %s | Origin: %s", 
			r.Header.Get("Upgrade"), r.Header.Get("Connection"), r.Header.Get("Origin"))
		
		// Check if this is a WebSocket upgrade request
		if !websocket.IsWebSocketUpgrade(r) {
			log.Printf("⚠️ [WARN] Non-WebSocket request rejected from %s (expected WebSocket upgrade)", clientIP)
			log.Printf("   💡 Tip: This endpoint only accepts WebSocket connections (ws:// or wss://)")
			http.Error(w, "This endpoint requires WebSocket upgrade", http.StatusBadRequest)
			return
		}
		
		log.Printf("🔄 [UPGRADE] Attempting WebSocket upgrade for %s...", clientIP)
		conn, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("❌ [ERROR] WebSocket upgrade FAILED from %s", clientIP)
			log.Printf("   Error details: %v", err)
			log.Printf("   Headers received - Upgrade: %s, Connection: %s", 
				r.Header.Get("Upgrade"), r.Header.Get("Connection"))
			log.Printf("   💡 Troubleshooting: Check CORS, firewall, or network connectivity")
			http.Error(w, fmt.Sprintf("WebSocket upgrade failed: %v", err), http.StatusBadRequest)
			return
		}
		
		log.Printf("✅ [UPGRADE] WebSocket upgrade successful for %s", clientIP)
		go handleLighting(conn) // Handle each connection in a goroutine
	})

	log.Printf("🚀 [STARTUP] Lighting WebSocket server starting...")
	log.Printf("   📍 Port: %s", port)
	log.Printf("   🛣️  Path: %s", path)
	log.Printf("   📜 LED Script: %s", runLedPath)
	
	// Get local IP addresses for better diagnostics
	log.Printf("   🌐 Network interfaces:")
	ifaces, err := net.Interfaces()
	if err == nil {
		for _, iface := range ifaces {
			addrs, err := iface.Addrs()
			if err != nil {
				continue
			}
			for _, addr := range addrs {
				if ipnet, ok := addr.(*net.IPNet); ok && !ipnet.IP.IsLoopback() {
					if ipnet.IP.To4() != nil {
						log.Printf("      - %s: %s:%s", iface.Name, ipnet.IP.String(), port)
					}
				}
			}
		}
	}
	
	// Try to bind and listen with detailed error handling
	listenAddr := ":" + port
	log.Printf("")
	log.Printf("🔌 [BIND] Attempting to bind to %s...", listenAddr)
	
	ln, err := net.Listen("tcp", listenAddr)
	if err != nil {
		log.Printf("❌ [FATAL] Failed to bind to %s", listenAddr)
		if strings.Contains(err.Error(), "address already in use") {
			log.Printf("   💡 Port %s is already in use!", port)
			log.Printf("   💡 Solution: Kill the process using port %s or use a different port:", port)
			log.Printf("      sudo lsof -i :%s", port)
			log.Printf("      sudo kill -9 <PID>")
			log.Printf("   💡 Or set PORT_LIGHTING environment variable to use a different port")
		} else if strings.Contains(err.Error(), "permission denied") {
			log.Printf("   💡 Permission denied! Ports below 1024 require root privileges")
			log.Printf("   💡 Solution: Use a port >= 1024 or run with sudo")
		} else {
			log.Printf("   💡 Error: %v", err)
		}
		log.Fatal(err)
	}
	
	log.Printf("✅ [BIND] Successfully bound to %s", listenAddr)
	log.Printf("")
	log.Printf("🎯 [READY] Server is now listening and accepting connections!")
	log.Printf("   📡 WebSocket URL: ws://<IP>:%s%s", port, path)
	log.Printf("   🏥 Health check: http://<IP>:%s/health", port)
	log.Printf("   ⏰ Started at: %s", time.Now().Format("2006-01-02 15:04:05"))
	log.Printf("   💡 Waiting for client connections...")
	log.Printf("")
	log.Printf("🔍 [DIAGNOSTICS] To test connectivity from another machine:")
	log.Printf("   curl http://100.93.225.61:%s/health", port)
	log.Printf("   curl http://192.168.6.164:%s/health", port)
	log.Printf("")
	log.Printf("⚠️  [FIREWALL] If connections fail, check firewall rules:")
	log.Printf("   sudo ufw status")
	log.Printf("   sudo ufw allow %s/tcp", port)
	log.Printf("")
	
	// Start serving with the listener we created
	log.Fatal(http.Serve(ln, nil))
}
