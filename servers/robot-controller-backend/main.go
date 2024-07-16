package main

/*
#cgo LDFLAGS: -L./rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_ultrasonic_data(char* input);
extern char* process_line_tracking_data(char* input);
*/
import "C"
import (
    "bytes"
    "crypto/tls"
    "encoding/json"
    "fmt"
    "io"
    "log"
    "net/http"
    "os"
    "os/exec"
    "runtime"
    "time"
    "unsafe"

    "github.com/gorilla/websocket"
    "github.com/joho/godotenv"
    "github.com/stianeikeland/go-rpio/v4"

    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/commands"
)

// GPIO interface and types definition
type GPIO interface {
    Open() error
    Close() error
    Pin(int) GPIOPin
}

type GPIOPin interface {
    Input()
    Output()
    Read() rpio.State
    High()
    Low()
}

// RealGPIO implementation for Raspberry Pi
type RealGPIO struct{}

func (r RealGPIO) Open() error {
    return rpio.Open()
}

func (r RealGPIO) Close() error {
    rpio.Close()
    return nil
}

func (r RealGPIO) Pin(pin int) GPIOPin {
    return rpio.Pin(pin)
}

// Initialize global variables
var (
    gpio        GPIO
    Low         rpio.State = 0
    High        rpio.State = 1
    execCommand            = exec.Command        // Declare execCommand for mocking in tests
    upgrader               = websocket.Upgrader{ // Add this block
        ReadBufferSize:  1024,
        WriteBufferSize: 1024,
        CheckOrigin: func(r *http.Request) bool {
            return true // Adjust this according to your CORS policy
        },
    }
)

func isRunningOnRaspberryPi() bool {
    return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}

func initGPIO() {
    if isRunningOnRaspberryPi() {
        gpio = RealGPIO{}
        Low = rpio.Low
        High = rpio.High
    } else {
        gpio = nil // No GPIO simulation in Go
    }
}

type LineTrackingData struct {
    IR01 int `json:"ir01"`
    IR02 int `json:"ir02"`
    IR03 int `json:"ir03"`
}

type UltrasonicData struct {
    Distance int `json:"distance"`
}

func logRequest(r *http.Request) {
    log.Printf("Received %s request for %s from %s\n", r.Method, r.URL, r.RemoteAddr)
}

func executePythonScript(scriptType, param1, param2 string) error {
    cmdArgs := []string{fmt.Sprintf("%s_control.py", scriptType), param1, param2}
    command := execCommand("python3", cmdArgs...)
    var out bytes.Buffer
    var stderr bytes.Buffer
    command.Stdout = &out
    command.Stderr = &stderr
    err := command.Run()
    if err != nil {
        log.Printf("Error executing Python script: %s\n", stderr.String())
        return err
    }
    log.Printf("Python script output: %s\n", out.String())
    return nil
}

func corsMiddleware(next http.Handler) http.Handler {
    return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
        w.Header().Set("Access-Control-Allow-Origin", "*")
        w.Header().Set("Access-Control-Allow-Methods", "POST, GET, OPTIONS, PUT, DELETE")
        w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

        if r.Method == "OPTIONS" {
            log.Printf("Received OPTIONS request from %s\n", r.RemoteAddr)
            w.WriteHeader(http.StatusOK)
            return
        }

        next.ServeHTTP(w, r)
    })
}

func handleConnections(w http.ResponseWriter, r *http.Request) {
    ws, err := upgrader.Upgrade(w, r, nil)
    if err != nil {
        log.Printf("Upgrade error: %v", err)
        return
    }
    defer ws.Close()

    // Send periodic pings to keep the connection alive
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

        // Handle commands here
        if command, ok := msg["command"].(string); ok {
            switch command {
            case "servo-horizontal", "servo-vertical", "move-up", "move-down", "move-left", "move-right":
                commands.ExecuteMotorCommand(commands.Command{
                    Command:   command,
                    Angle:     int(msg["angle"].(float64)),
                    RequestID: msg["request_id"].(string),
                })
            case "ultrasonic-sensor":
                handleWebSocketUltrasonicSensor(ws)
            case "line-tracking":
                handleWebSocketLineTracking(ws)
            default:
                log.Printf("Unknown command: %s\n", command)
            }
        }

        // Echo the message back to the client
        err = ws.WriteJSON(msg)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }
    }

    log.Printf("Connection closed")
}

func handleWebSocketUltrasonicSensor(ws *websocket.Conn) {
    if err := rpio.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        return
    }
    defer rpio.Close()

    trigger := rpio.Pin(27)
    echo := rpio.Pin(22)

    trigger.Output()
    echo.Input()

    for {
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

        // Process data using Rust
        input := fmt.Sprintf("%d", data.Distance)
        output := processUltrasonicData(input)
        log.Printf("Processed data: %s", output)

        err := ws.WriteJSON(data)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }

        time.Sleep(1 * time.Second) // Adjust the delay as needed
    }
}

func handleWebSocketLineTracking(ws *websocket.Conn) {
    if err := rpio.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        return
    }
    defer rpio.Close()

    ir01 := rpio.Pin(14)
    ir02 := rpio.Pin(15)
    ir03 := rpio.Pin(23)

    ir01.Input()
    ir02.Input()
    ir03.Input()

    for {
        data := LineTrackingData{
            IR01: int(ir01.Read()),
            IR02: int(ir02.Read()),
            IR03: int(ir03.Read()),
        }

        // Process data using Rust
        input := fmt.Sprintf("%d,%d,%d", data.IR01, data.IR02, data.IR03)
        output := processLineTrackingData(input)
        log.Printf("Processed data: %s", output)

        err := ws.WriteJSON(data)
        if err != nil {
            log.Printf("WriteJSON error: %v", err)
            break
        }

        time.Sleep(1 * time.Second) // Adjust the delay as needed
    }
}

func processUltrasonicData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_ultrasonic_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}

func processLineTrackingData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_line_tracking_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}

func handleCommand(w http.ResponseWriter, r *http.Request) {
    logRequest(r)

    var body bytes.Buffer
    _, err := body.ReadFrom(r.Body)
    if err != nil {
        log.Printf("Error reading body: %s\n", err)
        http.Error(w, "Error reading body", http.StatusBadRequest)
        return
    }
    r.Body = io.NopCloser(&body)

    var cmd commands.Command
    decoder := json.NewDecoder(r.Body)
    err = decoder.Decode(&cmd)
    if err != nil {
        log.Printf("Error decoding JSON: %s\n", err)
        http.Error(w, "Error decoding JSON", http.StatusBadRequest)
        return
    }

    log.Printf("Command received: %s", cmd.Command)

    switch cmd.Command {
    case "servo-horizontal", "servo-vertical":
        commands.ExecuteServoCommand(cmd)
    case "move-up", "move-down", "move-left", "move-right":
        commands.ExecuteMotorCommand(cmd)
    case "increase-speed", "decrease-speed", "buzz", "buzz-stop":
        log.Printf("Other command: %s", cmd.Command)
    default:
        log.Printf("Unknown command: %s\n", cmd.Command)
        http.Error(w, "Unknown command", http.StatusBadRequest)
    }

    log.Printf("Command executed: %s", cmd.Command)
    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}

func main() {
    log.SetOutput(os.Stdout)
    log.Printf("Starting server setup at %s\n", time.Now().Format(time.RFC3339))

    initGPIO()

    if err := godotenv.Load(".env"); err != nil {
        log.Printf("Error loading .env file: %s\n", err)
    } else {
        log.Printf(".env file loaded successfully")
    }

    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    if certPath == "" || keyPath == "" {
        log.Printf("CERT_PATH or KEY_PATH environment variable is not set\n")
        return
    }

    log.Printf("Using certificate: %s\n", certPath)
    log.Printf("Using key: %s\n", keyPath)

    mux := http.NewServeMux()
    mux.HandleFunc("/command", handleCommand)
    mux.HandleFunc("/ws", handleConnections) // Add WebSocket handler

    server := &http.Server{
        Addr:    ":8080",
        Handler: corsMiddleware(mux),
        TLSConfig: &tls.Config{
            MinVersion: tls.VersionTLS12,
        },
    }

    log.Printf("Starting secure server on :8080\n")
    if err := server.ListenAndServeTLS(certPath, keyPath); err != nil {
        log.Printf("Error starting server: %s\n", err)
    } else {
        log.Printf("Server started successfully")
    }
}
