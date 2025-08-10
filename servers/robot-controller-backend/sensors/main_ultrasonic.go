// File: main_ultrasonic.go
// Summary: 🧠 Omega 1 – Ultrasonic WebSocket Server (Pi 5 Compatible)
//
// This Go script uses the `periph.io` library to interface with an HC-SR04 ultrasonic sensor
// on a Raspberry Pi 5 without needing pigpio or background daemons. It runs a WebSocket server
// on port 8080 and streams live distance data (in cm, meters, inches, feet) to any connected clients.
//
// ✅ Highlights:
// - Uses `periph.io/x/host/v3` for direct GPIO control
// - Fully compatible with Raspberry Pi 5 (Ubuntu 24.04 or Raspberry Pi OS)
// - Daemonless operation (no `pigpiod` required)
// - Sends JSON payloads over WebSocket every 1 second
// - Graceful error handling (timeouts, invalid readings)
// - NEW: Sends a JSON welcome envelope on connect and responds to { "type": "ping" } with { "type": "pong" }
//
// 🔌 Wiring:
// - Trigger → GPIO27 (Physical Pin 13)
// - Echo    → GPIO22 (Physical Pin 15)
//
// Run it with:
// $ go run main_ultrasonic.go
//

package main

import (
	"encoding/json"
	"log"
	"math"
	"net/http"
	"os"
	"time"

	"github.com/gorilla/websocket"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/host/v3"
	"periph.io/x/host/v3/rpi"
)

type UltrasonicData struct {
	Status       string  `json:"status"`
	DistanceCM   int     `json:"distance_cm"`
	DistanceM    float64 `json:"distance_m"`
	DistanceInch float64 `json:"distance_inch"`
	DistanceFeet float64 `json:"distance_feet"`
	Error        string  `json:"error,omitempty"`
}

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

func measureDistance(trigger gpio.PinOut, echo gpio.PinIn) (int, error) {
	// Ensure low, then 10µs high pulse
	trigger.Out(gpio.Low)
	time.Sleep(2 * time.Microsecond)
	trigger.Out(gpio.High)
	time.Sleep(10 * time.Microsecond)
	trigger.Out(gpio.Low)

	// Wait for echo to go high (start)
	startWait := time.Now()
	for echo.Read() == gpio.Low {
		if time.Since(startWait) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	start := time.Now()

	// Wait for echo to go low (end)
	for echo.Read() == gpio.High {
		if time.Since(start) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	duration := time.Since(start)

	// Convert to cm (HC-SR04 ~58µs per cm round-trip)
	distanceCM := int(float64(duration.Microseconds()) / 58.0)
	if distanceCM <= 0 || distanceCM > 400 {
		return -1, os.ErrInvalid
	}
	return distanceCM, nil
}

func handleUltrasonicSensor(ws *websocket.Conn) {
	if _, err := host.Init(); err != nil {
		log.Printf("Failed to initialize periph: %v", err)
		_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: "Periph init failed"})
		return
	}

	trigger := rpi.P1_13 // GPIO27
	echo := rpi.P1_15    // GPIO22

	if err := trigger.Out(gpio.Low); err != nil {
		_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: "Trigger init failed"})
		return
	}
	if err := echo.In(gpio.PullNoChange, gpio.NoEdge); err != nil {
		_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: "Echo init failed"})
		return
	}

	// --- Welcome envelope so UI can mark connected immediately ---
	welcome := map[string]any{
		"status":  "connected",
		"service": "ultrasonic",
		"message": "Ultrasonic WebSocket connection established",
		"ts":      time.Now().UnixMilli(),
	}
	_ = ws.WriteJSON(welcome)

	// --- Concurrent reader: reply to JSON ping with pong ---
	done := make(chan struct{})
	go func() {
		defer close(done)
		for {
			_, msg, err := ws.ReadMessage()
			if err != nil {
				// client closed or read error
				return
			}
			var m map[string]any
			if err := json.Unmarshal(msg, &m); err == nil {
				if t, _ := m["type"].(string); t == "ping" {
					pong := map[string]any{"type": "pong", "ts": m["ts"]}
					_ = ws.WriteJSON(pong)
				}
			}
		}
	}()

	// --- Writer: stream distance once per second until disconnect ---
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-done:
			return
		case <-ticker.C:
			distanceCM, err := measureDistance(trigger, echo)
			if err != nil {
				_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: err.Error()})
				continue
			}

			meters := float64(distanceCM) / 100.0
			inches := float64(distanceCM) / 2.54
			feet := float64(distanceCM) / 30.48

			data := UltrasonicData{
				Status:       "success",
				DistanceCM:   distanceCM,
				DistanceM:    math.Round(meters*100) / 100,   // 2 decimals
				DistanceInch: math.Round(inches*100) / 100,   // 2 decimals
				DistanceFeet: math.Round(feet*100) / 100,     // 2 decimals
			}
			if err := ws.WriteJSON(data); err != nil {
				return
			}
		}
	}
}

func main() {
	log.SetOutput(os.Stdout)
	log.Println("🌐 Starting ultrasonic WebSocket server on :8080")

	http.HandleFunc("/ultrasonic", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("WebSocket upgrade error: %v", err)
			return
		}
		defer ws.Close()
		handleUltrasonicSensor(ws)
	})

	log.Fatal(http.ListenAndServe(":8080", nil))
}
