package main

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

    "github.com/joho/godotenv"
    "github.com/stianeikeland/go-rpio/v4"
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

// RealGPIO implements the GPIO interface for actual Raspberry Pi hardware
type RealGPIO struct{}

func (r RealGPIO) Open() error {
    return rpio.Open()
}

func (r RealGPIO) Close() error {
    return rpio.Close()
}

func (r RealGPIO) Pin(pin int) GPIOPin {
    return RealGPIOPin(rpio.Pin(pin))
}

type RealGPIOPin rpio.Pin

func (p RealGPIOPin) Input() {
    rpio.Pin(p).Input()
}

func (p RealGPIOPin) Output() {
    rpio.Pin(p).Output()
}

func (p RealGPIOPin) Read() rpio.State {
    return rpio.Pin(p).Read()
}

func (p RealGPIOPin) High() {
    rpio.Pin(p).High()
}

func (p RealGPIOPin) Low() {
    rpio.Pin(p).Low()
}

// Initialize global variables
var (
    gpio GPIO
    Low  rpio.State = 0
    High rpio.State = 1
    execCommand = exec.Command // Declare execCommand for mocking in tests
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

type Command struct {
    Command   string `json:"command"`
    Angle     int    `json:"angle,omitempty"`
    RequestID string `json:"request_id"`
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

func logCommand(cmd Command) {
    log.Printf("Executing command: %s with angle: %d and request_id: %s\n", cmd.Command, cmd.Angle, cmd.RequestID)
}

func executeServoCommand(cmd Command) {
    logCommand(cmd)

    var pin int
    if cmd.Command == "servo-horizontal" {
        pin = 17 // Example pin for horizontal servo
    } else if cmd.Command == "servo-vertical" {
        pin = 18 // Example pin for vertical servo
    } else {
        log.Printf("Unknown servo command: %s\n", cmd.Command)
        return
    }

    err := executePythonScript(pin, "HIGH")
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}

func executePythonScript(pin int, state string) error {
    cmdArgs := []string{"gpio_mock.py", fmt.Sprintf("%d", pin), state}
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

    var cmd Command
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
        executeServoCommand(cmd)
    case "move-up", "move-down", "move-left", "move-right":
        log.Printf("Movement command: %s", cmd.Command)
    case "increase-speed", "decrease-speed", "buzz", "buzz-stop":
        log.Printf("Other command: %s", cmd.Command)
    default:
        log.Printf("Unknown command: %s\n", cmd.Command)
        http.Error(w, "Unknown command", http.StatusBadRequest)
    }

    log.Printf("Command executed: %s", cmd.Command)
    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}

func handleLineTracking(w http.ResponseWriter, r *http.Request) {
    logRequest(r)

    err := executePythonScript(14, "HIGH")
    if err != nil {
        http.Error(w, "Error executing GPIO simulator", http.StatusInternalServerError)
        return
    }

    data := LineTrackingData{
        IR01: 1,
        IR02: 0,
        IR03: 1,
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(data)
}

func handleUltrasonicSensor(w http.ResponseWriter, r *http.Request) {
    logRequest(r)

    err := executePythonScript(27, "HIGH")
    if err != nil {
        http.Error(w, "Error executing GPIO simulator", http.StatusInternalServerError)
        return
    }

    data := UltrasonicData{Distance: 100}

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(data)
}

func boolToInt(b bool) int {
    if b {
        return 1
    }
    return 0
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
    mux.HandleFunc("/line-tracking", handleLineTracking)
    mux.HandleFunc("/ultrasonic-sensor", handleUltrasonicSensor)

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
