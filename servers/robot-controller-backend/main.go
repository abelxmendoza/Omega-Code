package main

import (
    "bytes"
    "crypto/tls"
    "encoding/json"
    "fmt"
    "log"
    "net/http"
    "os"
    "os/exec"
    "time"
    "io"

    "github.com/joho/godotenv"
)

type Command struct {
    Command   string `json:"command"`
    Angle     int    `json:"angle,omitempty"`
    RequestID string `json:"request_id"` // Add this field to hold the unique identifier
}

func logRequest(r *http.Request) {
    log.Printf("Received %s request for %s from %s\n", r.Method, r.URL, r.RemoteAddr)
}

func logCommand(cmd Command) {
    log.Printf("Executing command: %s with angle: %d and request_id: %s\n", cmd.Command, cmd.Angle, cmd.RequestID)
}

func executeServoCommand(cmd Command) {
    logCommand(cmd)

    var servoType string
    if cmd.Command == "servo-horizontal" {
        servoType = "horizontal"
    } else if cmd.Command == "servo-vertical" {
        servoType = "vertical"
    } else {
        log.Printf("Unknown servo command: %s\n", cmd.Command)
        return
    }

    cmdArgs := []string{"servo_control.py", servoType, fmt.Sprintf("%d", cmd.Angle)}
    command := exec.Command("python3", cmdArgs...)
    var out bytes.Buffer
    var stderr bytes.Buffer
    command.Stdout = &out
    command.Stderr = &stderr
    err := command.Run()
    if err != nil {
        log.Printf("Error executing Python script: %s\n", stderr.String())
    } else {
        log.Printf("Python script output: %s\n", out.String())
    }
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

func main() {
    log.SetOutput(os.Stdout)
    log.Printf("Starting server setup at %s\n", time.Now().Format(time.RFC3339))

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
