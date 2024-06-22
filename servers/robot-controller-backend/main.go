package main

import (
    "bytes"
    "crypto/tls"
    "encoding/json"
    "fmt"
    "io/ioutil"
    "log"
    "net/http"
    "os"
    "os/exec"
    "time"

    "github.com/joho/godotenv"
)

type Command struct {
    Command string `json:"command"`
    Angle   int    `json:"angle,omitempty"`
}

func logRequest(r *http.Request) {
    log.Printf("Received %s request for %s from %s\n", r.Method, r.URL, r.RemoteAddr)
}

func logCommand(cmd Command) {
    log.Printf("Executing command: %s with angle: %d\n", cmd.Command, cmd.Angle)
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
    err := command.Run()
    if err != nil {
        log.Printf("Error executing Python script: %s\n", err)
    }
}

func corsMiddleware(next http.Handler) http.Handler {
    return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
        w.Header().Set("Access-Control-Allow-Origin", "*")
        w.Header().Set("Access-Control-Allow-Methods", "POST, GET, OPTIONS, PUT, DELETE")
        w.Header().Set("Access-Control-Allow-Headers", "Content-Type, Authorization")

        if r.Method == "OPTIONS" {
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
    r.Body = ioutil.NopCloser(&body)

    var cmd Command
    decoder := json.NewDecoder(r.Body)
    err = decoder.Decode(&cmd)
    if err != nil {
        log.Printf("Error decoding JSON: %s\n", err)
        http.Error(w, "Error decoding JSON", http.StatusBadRequest)
        return
    }

    if cmd.Command == "servo-horizontal" || cmd.Command == "servo-vertical" {
        executeServoCommand(cmd)
    } else {
        log.Printf("Unknown command: %s\n", cmd.Command)
        http.Error(w, "Unknown command", http.StatusBadRequest)
    }

    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}

func main() {
    logFile, err := os.OpenFile("server.log", os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0666)
    if err != nil {
        fmt.Printf("Failed to open log file: %s\n", err)
        return
    }
    log.SetOutput(logFile)
    log.Printf("Server started at %s\n", time.Now().Format(time.RFC3339))

    if err := godotenv.Load("../../config/.env"); err != nil {
        log.Printf("Error loading .env file: %s\n", err)
    }

    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    if certPath == "" || keyPath == "" {
        log.Printf("CERT_PATH or KEY_PATH environment variable is not set\n")
        return
    }

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
    }
}
