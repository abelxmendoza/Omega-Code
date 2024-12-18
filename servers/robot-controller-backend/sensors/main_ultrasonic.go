/*
# File: /Omega-Code/servers/robot-controller-backend/sensors/main_ultrasonic.go

Ultrasonic Sensor Control and Data Processing with WebSockets

This script handles the ultrasonic sensor functionality for a Raspberry Pi,
including reading sensor data, processing it using Rust integration, and
sending the data through WebSockets.
*/

package main

import (
    "log"
    "net/http"
    "time"
    "github.com/gorilla/websocket"
    "github.com/stianeikeland/go-rpio/v4"
    "fmt"
    "math"
)

type UltrasonicData struct {
    DistanceCM    int     `json:"distance_cm"`
    DistanceM     float64 `json:"distance_m"`
    DistanceInch  float64 `json:"distance_inch"`
    DistanceFeet  float64 `json:"distance_feet"`
}

var upgrader = websocket.Upgrader{
    ReadBufferSize:  1024,
    WriteBufferSize: 1024,
    CheckOrigin: func(r *http.Request) bool {
        return true // Adjust this according to your CORS policy
    },
}

func handleUltrasonicSensor(ws *websocket.Conn) {
    // Open GPIO for reading sensor data
    if err := rpio.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        return
    }
    defer rpio.Close() // Ensure GPIO is closed after the function completes

    // Define GPIO pins for trigger and echo
    trigger := rpio.Pin(27)
    echo := rpio.Pin(22)

    trigger.Output() // Set trigger pin as output
    echo.Input()     // Set echo pin as input

    ticker := time.NewTicker(1 * time.Second) // Adjust the interval as needed
    defer ticker.Stop()

    for {
        select {
        case <-ticker.C:
            // Send trigger pulse
            trigger.Low()
            time.Sleep(2 * time.Microsecond)
            trigger.High()
            time.Sleep(10 * time.Microsecond)
            trigger.Low()

            // Measure the duration of the echo pulse
            start := time.Now()
            for echo.Read() == rpio.Low {
            }
            start = time.Now()

            for echo.Read() == rpio.High {
            }
            duration := time.Since(start)
            distanceCM := int(duration.Seconds() * 17150) // Calculate distance in cm

            // Convert to other units
            distanceM := float64(distanceCM) / 100
            distanceInch := float64(distanceCM) / 2.54
            distanceFeet := distanceInch / 12

            // Log distances
            log.Printf("Measured distance: %d cm, %.2f m, %.2f inches, %.2f feet", distanceCM, distanceM, distanceInch, distanceFeet)

            // Create a data struct with the measured distances
            data := UltrasonicData{
                DistanceCM:    distanceCM,
                DistanceM:     distanceM,
                DistanceInch:  math.Round(distanceInch*100) / 100, // Round to 2 decimal places
                DistanceFeet:  math.Round(distanceFeet*100) / 100, // Round to 2 decimal places
            }

            // Send the data through WebSocket
            err := ws.WriteJSON(data)
            if err != nil {
                log.Printf("WriteJSON error: %v", err)
                return
            }
        }
    }
}

func main() {
    log.SetOutput(os.Stdout)
    log.Printf("Starting ultrasonic sensor WebSocket server at %s\n", time.Now().Format(time.RFC3339))

    http.HandleFunc("/ultrasonic", func(w http.ResponseWriter, r *http.Request) {
        ws, err := upgrader.Upgrade(w, r, nil)
        if err != nil {
            log.Printf("Upgrade error: %v", err)
            return
        }
        defer ws.Close()

        handleUltrasonicSensor(ws)
    })

    addr := ":8080"
    log.Printf("Starting server on %s\n", addr)
    log.Fatal(http.ListenAndServe(addr, nil))
}
