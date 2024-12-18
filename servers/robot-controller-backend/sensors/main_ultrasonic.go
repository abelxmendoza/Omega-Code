// File: /Omega-Code/servers/robot-controller-backend/main_ultrasonic.go


/*
Ultrasonic Sensor Control and Data Processing with WebSockets

This script handles the ultrasonic sensor functionality for a Raspberry Pi,
including reading sensor data, processing it using Rust integration, and 
sending the data through WebSockets.

Functions:
- processUltrasonicData: Processes ultrasonic sensor data using Rust code.
- handleUltrasonicSensor: Handles WebSocket connections to read and process ultrasonic sensor data.

Types:
- UltrasonicData: Struct to represent the distance measured by the ultrasonic sensor.

*/

package main

import (
    "encoding/json"
    "log"
    "time"
    "github.com/gorilla/websocket"
    "github.com/stianeikeland/go-rpio"
)

// Rust integration
/*
#cgo LDFLAGS: -L/home/omega1/Documents/code/Omega-Code/servers/robot-controller-backend/rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_ultrasonic_data(char* input);
*/
import "C"
import (
    "fmt"
    "unsafe"
)

func processUltrasonicData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_ultrasonic_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}

type UltrasonicData struct {
    Distance int `json:"distance"`
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
            distance := int(duration.Seconds() * 17150) // Calculate distance in cm

            // Create a data struct with the measured distance
            data := UltrasonicData{Distance: distance}

            // Process the data using Rust
            input := fmt.Sprintf("%d", data.Distance)
            output := processUltrasonicData(input)
            log.Printf("Processed data: %s", output)

            // Send the data through WebSocket
            err := ws.WriteJSON(data)
            if err != nil {
                log.Printf("WriteJSON error: %v", err)
                return
            }

            // Optional: Execute the Python script
            cmd := exec.Command("python3", "ultrasonic_sensor.py")
            err = cmd.Run()
            if err != nil {
                log.Printf("Error executing ultrasonic sensor Python script: %s\n", err)
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
