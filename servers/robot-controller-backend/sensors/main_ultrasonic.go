// File: main_ultrasonic.go
// Summary: WebSocket ultrasonic server using periph.io (daemonless, Pi 5-compatible)

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
	"periph.io/x/conn/v3/physic"
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
	trigger.Out(gpio.Low)
	time.Sleep(2 * time.Microsecond)
	trigger.Out(gpio.High)
	time.Sleep(10 * time.Microsecond)
	trigger.Out(gpio.Low)

	// Wait for echo to go high
	start := time.Now()
	for echo.Read() == gpio.Low {
		if time.Since(start) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	start = time.Now()

	// Wait for echo to go low
	for echo.Read() == gpio.High {
		if time.Since(start) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	duration := time.Since(start)
	distanceCM := int(float64(duration.Nanoseconds()) / 1e3 / 58.0)

	if distanceCM <= 0 || distanceCM > 400 {
		return -1, os.ErrInvalid
	}
	return distanceCM, nil
}

func handleUltrasonicSensor(ws *websocket.Conn) {
	if _, err := host.Init(); err != nil {
		log.Printf("Failed to initialize periph: %v", err)
		ws.WriteJSON(UltrasonicData{Status: "error", Error: "Periph init failed"})
		return
	}

	trigger := rpi.P1_13 // GPIO27
	echo := rpi.P1_15    // GPIO22

	if err := trigger.Out(gpio.Low); err != nil {
		ws.WriteJSON(UltrasonicData{Status: "error", Error: "Trigger init failed"})
		return
	}
	if err := echo.In(gpio.PullNoChange, gpio.NoEdge); err != nil {
		ws.WriteJSON(UltrasonicData{Status: "error", Error: "Echo init failed"})
		return
	}

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		distanceCM, err := measureDistance(trigger, echo)
		if err != nil {
			ws.WriteJSON(UltrasonicData{Status: "error", Error: err.Error()})
			continue
		}
		payload := UltrasonicData{
			Status:       "success",
			DistanceCM:   distanceCM,
			DistanceM:    math.Round(float64(distanceCM)/100*100) / 100,
			DistanceInch: math.Round(float64(distanceCM)/2.54*100) / 100,
			DistanceFeet: math.Round(float64(distanceCM)/30.48*100) / 100,
		}
		ws.WriteJSON(payload)
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
