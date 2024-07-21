// File: /Omega-Code/servers/robot-controller-backend/core/core.go

/*
Package core provides the main functionality for handling WebSocket connections and routing commands for the robot controller.
It includes functions for handling connections, ultrasonic sensors, line tracking, LED control, and starting the Python video server.
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
    "unsafe"

    "github.com/gorilla/websocket"
    "github.com/joho/godotenv"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/commands"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/common"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/gpio"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/rust_integration"
)

var (
    ExecCommand = exec.Command // Declare ExecCommand for mocking in tests
    Upgrader    = websocket.Upgrader{
        ReadBufferSize:  1024,
        WriteBufferSize: 1024,
        CheckOrigin: func(r *http.Request) bool {
            return true // Adjust this according to your CORS policy
        },
    }
)

// HandleConnections manages WebSocket connections and routes commands to appropriate handlers.
func HandleConnections(w http.ResponseWriter, r *http.Request) {
    ws, err := Upgrader.Upgrade(w, r, nil)
    if err != nil {
        log.Printf("Upgrade error: %v", err)
        return
    }
    defer ws.Close()

    ticker := time.NewTicker(30 * time.Second)
    defer ticker.Stop()

    go func() {
        for {
            <-ticker.C
            if err := ws.WriteMessage(websocket.PingMessage, []byte{}); err != nil {
                log.Printf("Error sending ping: %v", err)
                return
            }
        }
    }()

    for {
        var msg map[string]interface{}
        err := ws.ReadJSON(&msg)
        if err != nil {
            if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
                log.Printf("Unexpected close error: %v", err)
            } else {
                log.Printf("ReadJSON error: %v", err)
            }
            break
        }
        log.Printf("Received: %v", msg)

        if command, ok := msg["command"].(string); ok {
            switch command {
            case "camera-left", "camera-right", "camera-up", "camera-down":
                commands.ExecuteServoCommand(commands.Command{
                    Command:   command,
                    Angle:     int(msg["angle"].(float64)),
                    RequestID: msg["request_id"].(string),
                })
            case "ultrasonic-sensor":
                HandleWebSocketUltrasonicSensor(ws)
            case "line-tracking":
                HandleWebSocketLineTracking(ws)
            case "led-control":
                HandleWebSocketLEDControl(ws, msg)
            case "camera-stream":
                HandleWebSocketCameraStream(ws)
            default:
                log.Printf("Unknown command: %s\n", command)
            }
        }

        err = ws.WriteJSON(msg)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }
    }

    log.Printf("Connection closed")
}

// HandleWebSocketUltrasonicSensor handles the ultrasonic sensor commands over WebSocket.
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
        distance := int(duration.Seconds() * 17150)

        data := commands.UltrasonicData{Distance: distance}

        input := fmt.Sprintf("%d", data.Distance)
        output := rust_integration.ProcessUltrasonicData(input)
        log.Printf("Processed data: %s", output)

        err := ws.WriteJSON(data)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }

        time.Sleep(1 * time.Second)
    }
}

// HandleWebSocketLineTracking handles the line tracking sensor commands over WebSocket.
func HandleWebSocketLineTracking(ws *websocket.Conn) {
    if err := gpio.GpioInterface.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        return
    }
    defer gpio.GpioInterface.Close()

    ir01 := gpio.GpioInterface.Pin(14)
    ir02 := gpio.GpioInterface.Pin(15)
    ir03 := gpio.GpioInterface.Pin(23)

    ir01.Input()
    ir02.Input()
    ir03.Input()

    for {
        data := commands.LineTrackingData{
            IR01: int(ir01.Read()),
            IR02: int(ir02.Read()),
            IR03: int(ir03.Read()),
        }

        input := fmt.Sprintf("%d,%d,%d", data.IR01, data.IR02, data.IR03)
        output := rust_integration.ProcessLineTrackingData(input)
        log.Printf("Processed data: %s", output)

        err := ws.WriteJSON(data)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }

        time.Sleep(1 * time.Second)
    }
}

// HandleWebSocketLEDControl handles the LED control commands over WebSocket.
func HandleWebSocketLEDControl(ws *websocket.Conn, msg map[string]interface{}) {
    color := int(msg["color"].(float64))
    mode := msg["mode"].(string)
    pattern := msg["pattern"].(string)
    interval := int(msg["interval"].(float64))

    err := common.ExecutePythonScript("led", fmt.Sprintf("%x", color), mode, pattern, fmt.Sprintf("%d", interval))
    if err != nil {
        log.Printf("Error executing LED control script: %s\n", err)
    }
}

// HandleWebSocketCameraStream handles the camera stream commands over WebSocket.
func HandleWebSocketCameraStream(ws *websocket.Conn) {
    cmd := exec.Command("python3", "video/video_server.py")
    err := cmd.Start()
    if err != nil {
        log.Printf("Error starting camera stream server: %s\n", err)
    }
}

// StartPythonVideoServer starts the Python video server.
func StartPythonVideoServer() {
    cmd := exec.Command("python3", "video/video_server.py")
    cmd.Stdout = os.Stdout
    cmd.Stderr = os.Stderr
    err := cmd.Start()
    if err != nil {
        log.Fatalf("Error starting Python video server: %s", err)
    }
    log.Printf("Started Python video server with PID %d", cmd.Process.Pid)
}

// StartServer initializes and starts the HTTP server with TLS support.
func StartServer() {
    log.SetOutput(os.Stdout)
    log.Printf("Starting server setup at %s\n", time.Now().Format(time.RFC3339))

    commands.InitGPIO()

    if err := godotenv.Load(".env"); err != nil {
        log.Printf("Error loading .env file: %s", err)
    } else {
        log.Printf(".env file loaded successfully")
    }

    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    if certPath == "" || keyPath == "" {
        log.Printf("CERT_PATH or KEY_PATH environment variable is not set")
        return
    }

    log.Printf("Using certificate: %s", certPath)
    log.Printf("Using key: %s", keyPath)

    mux := http.NewServeMux()
    mux.HandleFunc("/command", commands.HandleCommand)
    mux.HandleFunc("/ws", HandleConnections)

    server := &http.Server{
        Addr:    ":8080",
        Handler: common.CorsMiddleware(mux),
        TLSConfig: &tls.Config{
            MinVersion: tls.VersionTLS12,
        },
    }

    log.Printf("Starting secure server on :8080")
    if err := server.ListenAndServeTLS(certPath, keyPath); err != nil {
        log.Printf("Error starting server: %s", err)
    } else {
        log.Printf("Server started successfully")
    }
}
