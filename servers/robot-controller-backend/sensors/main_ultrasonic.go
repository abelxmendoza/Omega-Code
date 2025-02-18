/*
# File: /Omega-Code/servers/robot-controller-backend/sensors/main_ultrasonic.go
# Summary:
Ultrasonic Sensor Control and Data Processing with WebSockets
*/

package main

import (
	"encoding/json"
	"log"
	"math"
	"net/http"
	"os"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stianeikeland/go-rpio/v4"
)

// UltrasonicData defines the structure of the distance data sent via WebSocket
type UltrasonicData struct {
	Status       string  `json:"status"`        // Operation status
	DistanceCM   int     `json:"distance_cm"`   // Distance in centimeters
	DistanceM    float64 `json:"distance_m"`    // Distance in meters
	DistanceInch float64 `json:"distance_inch"` // Distance in inches
	DistanceFeet float64 `json:"distance_feet"` // Distance in feet
	Error        string  `json:"error"`         // Error message, if any
}

// WebSocket upgrader with CORS policy
var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true // Adjust for stricter CORS in production
	},
}

// handleUltrasonicSensor reads sensor data and sends it via WebSocket
func handleUltrasonicSensor(ws *websocket.Conn) {
	// Initialize GPIO
	if err := rpio.Open(); err != nil {
		log.Printf("Error opening GPIO: %v", err)
		ws.WriteJSON(UltrasonicData{
			Status: "error",
			Error:  "Failed to initialize GPIO",
		})
		return
	}
	defer rpio.Close()

	// Define GPIO pins for trigger and echo
	trigger := rpio.Pin(27)
	echo := rpio.Pin(22)
	trigger.Output()
	echo.Input()

	ticker := time.NewTicker(1 * time.Second) // Measurement interval
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			trigger.Low()
			time.Sleep(2 * time.Microsecond)
			trigger.High()
			time.Sleep(10 * time.Microsecond)
			trigger.Low()

			start := time.Now()
			for echo.Read() == rpio.Low {
				if time.Since(start) > 1*time.Second {
					ws.WriteJSON(UltrasonicData{
						Status: "error",
						Error:  "Echo signal timeout",
					})
					return
				}
			}
			start = time.Now()

			for echo.Read() == rpio.High {
				if time.Since(start) > 1*time.Second {
					ws.WriteJSON(UltrasonicData{
						Status: "error",
						Error:  "Echo signal timeout",
					})
					return
				}
			}

			duration := time.Since(start)
			distanceCM := int(duration.Seconds() * 17150)

			if distanceCM <= 0 || distanceCM > 400 {
				ws.WriteJSON(UltrasonicData{
					Status: "error",
					Error:  "Invalid distance measurement",
				})
				continue
			}

			// Convert distances
			distanceM := float64(distanceCM) / 100
			distanceInch := float64(distanceCM) / 2.54
			distanceFeet := distanceInch / 12

			// Log and send distance data
			log.Printf("Distance: %d cm, %.2f m, %.2f inches, %.2f feet", distanceCM, distanceM, distanceInch, distanceFeet)

			data := UltrasonicData{
				Status:       "success",
				DistanceCM:   distanceCM,
				DistanceM:    math.Round(distanceM*100) / 100,
				DistanceInch: math.Round(distanceInch*100) / 100,
				DistanceFeet: math.Round(distanceFeet*100) / 100,
			}
			if err := ws.WriteJSON(data); err != nil {
				log.Printf("WebSocket WriteJSON error: %v", err)
				return
			}
		}
	}
}

func main() {
	log.SetOutput(os.Stdout)
	log.Println("Starting ultrasonic sensor WebSocket server...")

	http.HandleFunc("/ultrasonic", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("WebSocket upgrade error: %v", err)
			return
		}
		defer ws.Close()
		handleUltrasonicSensor(ws)
	})

	addr := ":8080"
	log.Printf("Server listening on %s", addr)
	log.Fatal(http.ListenAndServe(addr, nil))
}
