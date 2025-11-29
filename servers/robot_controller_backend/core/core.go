// File: /Omega-Code/servers/robot_controller_backend/core/core.go

/*
Package core provides the main functionality for handling WebSocket connections
and routing commands for the robot controller. It includes functions for handling
connections, ultrasonic sensors, line tracking, LED control, and starting the
Python video server.
*/

package core

import (
    "crypto/tls"
    "fmt"
    "log"
    "net/http"
    "os"
    "os/exec"
    "time"

    "github.com/gorilla/websocket"
    "github.com/joho/godotenv"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/commands"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/common"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/gpio"
)

var (
    ExecCommand = exec.Command
    Upgrader    = websocket.Upgrader{
        ReadBufferSize:  1024,
        WriteBufferSize: 1024,
        CheckOrigin: func(r *http.Request) bool {
            return true // Adjust this according to your CORS policy
        },
    }
    PiIP = "127.0.0.1" // Default IP, overwritten by environment variable
)

// InitializeCore loads environment variables and initializes key components.
func InitializeCore() {
    if err := godotenv.Load(".env"); err != nil {
        log.Printf("Error loading .env file: %s", err)
    }

    // Load the Raspberry Pi IP from the environment variable
    PiIP = os.Getenv("PI_IP")
    if PiIP == "" {
        log.Println("PI_IP not set in environment, using default 127.0.0.1")
        PiIP = "127.0.0.1"
    } else {
        log.Printf("Using PI_IP: %s", PiIP)
    }
}

// HandleConnections manages WebSocket connections and routes commands.
func HandleConnections(w http.ResponseWriter, r *http.Request) {
    ws, err := Upgrader.Upgrade(w, r, nil)
    if err != nil {
        log.Printf("Upgrade error: %v", err)
        return
    }
    defer ws.Close()

    for {
        var msg map[string]interface{}
        err := ws.ReadJSON(&msg)
        if err != nil {
            log.Printf("ReadJSON error: %v", err)
            break
        }

        if command, ok := msg["command"].(string); ok {
            switch command {
            case "ultrasonic-sensor":
                go HandleWebSocketUltrasonicSensor(ws)
            default:
                log.Printf("Unknown command: %s\n", command)
            }
        }
    }
}

// HandleWebSocketUltrasonicSensor sends ultrasonic sensor data over WebSocket.
func HandleWebSocketUltrasonicSensor(ws *websocket.Conn) {
    if err := gpio.GpioInterface.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        return
    }
    defer gpio.GpioInterface.Close()

    trigger := gpio.GpioInterface.Pin(27)
    echo := gpio.GpioInterface.Pin(22)

    trigger.Output()
    echo.Input()

    for {
        trigger.Low()
        time.Sleep(2 * time.Microsecond)
        trigger.High()
        time.Sleep(10 * time.Microsecond)
        trigger.Low()

        start := time.Now()
        for echo.Read() == gpio.Low {
        }
        start = time.Now()

        for echo.Read() == gpio.High {
        }
        duration := time.Since(start)
        distanceCM := int(duration.Seconds() * 17150)

        data := map[string]interface{}{
            "distance_cm":   distanceCM,
            "distance_m":    float64(distanceCM) / 100,
            "distance_inch": float64(distanceCM) / 2.54,
            "distance_feet": float64(distanceCM) / 30.48,
        }

        err := ws.WriteJSON(data)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }
        time.Sleep(1 * time.Second)
    }
}

// StartServer initializes and starts the HTTP server.
func StartServer() {
    InitializeCore() // Load environment variables

    log.Printf("Starting server setup at %s\n", time.Now().Format(time.RFC3339))
    mux := http.NewServeMux()
    mux.HandleFunc("/ws", HandleConnections)

    server := &http.Server{
        Addr:    fmt.Sprintf("%s:8080", PiIP), // Dynamically use the IP from environment
        Handler: mux,
        TLSConfig: &tls.Config{
            MinVersion: tls.VersionTLS12,
        },
    }

    log.Printf("Starting server on %s:8080", PiIP)
    log.Fatal(server.ListenAndServe())
}