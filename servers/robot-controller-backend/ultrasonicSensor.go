// +build rpi

package main

import (
    "encoding/json"
    "log"
    "net/http"
    "os/exec"
    "time"
    "github.com/stianeikeland/go-rpio"
)

type UltrasonicData struct {
    Distance int `json:"distance"`
}

func handleUltrasonicSensor(w http.ResponseWriter, r *http.Request) {
    logRequest(r)

    if err := rpio.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        http.Error(w, "Error opening GPIO", http.StatusInternalServerError)
        return
    }
    defer rpio.Close()

    trigger := rpio.Pin(27)
    echo := rpio.Pin(22)

    trigger.Output()
    echo.Input()

    trigger.Low()
    time.Sleep(2 * time.Microsecond)
    trigger.High()
    time.Sleep(10 * time.Microsecond)
    trigger.Low()

    start := time.Now()
    for echo.Read() == rpio.Low {
    }
    start = time.Now()

    for echo.Read() == rpio.High {
    }
    duration := time.Since(start)
    distance := int(duration.Seconds() * 17150) // distance in cm

    data := UltrasonicData{Distance: distance}

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(data)

    // Optional: Execute the Python script
    cmd := exec.Command("python3", "ultrasonic_sensor.py")
    err := cmd.Run()
    if err != nil {
        log.Printf("Error executing ultrasonic sensor Python script: %s\n", err)
    }
}
