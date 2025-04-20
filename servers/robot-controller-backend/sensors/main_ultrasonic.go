// File: /Omega-Code/servers/robot-controller-backend/sensors/main_ultrasonic.go
// Summary: Ultrasonic Sensor Control and Data Streaming using pigpiod via WebSockets

package main

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mitchellh/go-ps"
	"github.com/zeromq/goczmq"
	"github.com/joan2937/pigpio-client-go/pigpio"
)

// UltrasonicData holds sensor output in multiple units
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

const (
	TRIG = 27
	ECHO = 22
	TIMEOUT = 1000000 // 1 second in microseconds
)

func startPigpiodIfNeeded() {
	found := false
	processes, err := ps.Processes()
	if err == nil {
		for _, proc := range processes {
			if proc.Executable() == "pigpiod" {
				found = true
				break
			}
		}
	}
	if !found {
		log.Println("[INFO] pigpiod not running. Starting pigpiod...")
		err := os.StartProcess("/usr/local/bin/pigpiod", []string{"pigpiod"}, &os.ProcAttr{
			Files: []*os.File{os.Stdin, os.Stdout, os.Stderr},
		})
		if err != nil {
			log.Fatalf("[ERROR] Failed to start pigpiod: %v", err)
		}
		time.Sleep(2 * time.Second) // give pigpiod time to initialize
	}
}

func measureDistance(pi *pigpio.Pigpio) (int, error) {
	_ = pi.Write(TRIG, pigpio.Low)
	time.Sleep(2 * time.Microsecond)
	_ = pi.Write(TRIG, pigpio.High)
	time.Sleep(10 * time.Microsecond)
	_ = pi.Write(TRIG, pigpio.Low)

	startTick := time.Now()
	for pi.Read(ECHO) == 0 {
		if time.Since(startTick) > time.Second {
			return -1, fmt.Errorf("echo start timeout")
		}
	}
	start := time.Now()

	for pi.Read(ECHO) == 1 {
		if time.Since(start) > time.Second {
			return -1, fmt.Errorf("echo end timeout")
		}
	}
	end := time.Now()
	pulseDuration := end.Sub(start).Seconds()
	distanceCM := int(pulseDuration * 17150)

	if distanceCM <= 0 || distanceCM > 400 {
		return -1, fmt.Errorf("invalid distance: %d cm", distanceCM)
	}
	return distanceCM, nil
}

func handleUltrasonicSensor(ws *websocket.Conn) {
	startPigpiodIfNeeded()
	pi, err := pigpio.NewPigpio("localhost", "8888")
	if err != nil {
		log.Printf("[ERROR] Pigpio connection: %v", err)
		_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: "Failed to connect to pigpiod"})
		return
	}
	defer pi.Close()

	_ = pi.SetMode(TRIG, pigpio.Output)
	_ = pi.SetMode(ECHO, pigpio.Input)

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		distanceCM, err := measureDistance(pi)
		if err != nil {
			_ = ws.WriteJSON(UltrasonicData{Status: "error", Error: err.Error()})
			continue
		}
		distanceM := float64(distanceCM) / 100
		distanceInch := float64(distanceCM) / 2.54
		distanceFeet := distanceInch / 12

		payload := UltrasonicData{
			Status:       "success",
			DistanceCM:   distanceCM,
			DistanceM:    math.Round(distanceM*100) / 100,
			DistanceInch: math.Round(distanceInch*100) / 100,
			DistanceFeet: math.Round(distanceFeet*100) / 100,
		}
		_ = ws.WriteJSON(payload)
	}
}

func main() {
	log.SetOutput(os.Stdout)
	log.Println("🌐 Starting ultrasonic WebSocket server on :8080")

	http.HandleFunc("/ultrasonic", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("[ERROR] WebSocket upgrade: %v", err)
			return
		}
		defer ws.Close()
		handleUltrasonicSensor(ws)
	})

	log.Fatal(http.ListenAndServe(":8080", nil))
}
