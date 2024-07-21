// File: /Omega-Code/servers/robot-controller-backend/commands/commands.go

/*
Package commands provides functions to handle various commands for the robot controller.
It includes handlers for WebSocket connections, ultrasonic sensors, line tracking, LED control, and camera streams.
*/

package commands

import (
    "fmt"
    "log"
    "net/http"
    "os/exec"
    "time"

    "github.com/gorilla/websocket"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/common"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/rust_integration"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/gpio"
)

var upgrader = websocket.Upgrader{
    ReadBufferSize:  1024,
    WriteBufferSize: 1024,
    CheckOrigin: func(r *http.Request) bool {
        return true // Adjust this according to your CORS policy
    },
}

// HandleConnections manages WebSocket connections and routes commands to appropriate handlers.
func HandleConnections(w http.ResponseWriter, r *http.Request) {
    ws, err := upgrader.Upgrade(w, r, nil)
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
                ExecuteServoCommand(Command{
                    Command:   command,
                    Angle:     int(msg["angle"].(float64)),
                    RequestID: msg["request_id"].(string),
                })
            case "ultrasonic-sensor":
                handleWebSocketUltrasonicSensor(ws)
            case "line-tracking":
                handleWebSocketLineTracking(ws)
            case "led-control":
                handleWebSocketLEDControl(ws, msg)
            case "camera-stream":
                handleWebSocketCameraStream(ws)
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

// handleWebSocketUltrasonicSensor handles the ultrasonic sensor commands over WebSocket.
func handleWebSocketUltrasonicSensor(ws *websocket.Conn) {
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

        data := UltrasonicData{Distance: distance}

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

// handleWebSocketLineTracking handles the line tracking sensor commands over WebSocket.
func handleWebSocketLineTracking(ws *websocket.Conn) {
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
        data := LineTrackingData{
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

// handleWebSocketLEDControl handles the LED control commands over WebSocket.
func handleWebSocketLEDControl(ws *websocket.Conn, msg map[string]interface{}) {
    color := int(msg["color"].(float64))
    mode := msg["mode"].(string)
    pattern := msg["pattern"].(string)
    interval := int(msg["interval"].(float64))

    err := common.ExecutePythonScript("led", fmt.Sprintf("%x", color), mode, pattern, fmt.Sprintf("%d", interval))
    if err != nil {
        log.Printf("Error executing LED control script: %s\n", err)
    }
}

// handleWebSocketCameraStream handles the camera stream commands over WebSocket.
func handleWebSocketCameraStream(ws *websocket.Conn) {
    cmd := exec.Command("python3", "video/video_server.py")
    err := cmd.Start()
    if err != nil {
        log.Printf("Error starting camera stream server: %s\n", err)
    }
}
