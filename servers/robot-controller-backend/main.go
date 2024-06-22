package main

import (
    "bytes"
    "crypto/tls"
    "encoding/json"
    "fmt"
    "io/ioutil"
    "net/http"
    "os"
    "os/exec"

    "github.com/joho/godotenv"
)

// Summary
// The provided Go code handles commands sent from a frontend application to control LEDs and servos on a robot.
// It also includes handling of movement commands and other robot controls. The code utilizes environment variables
// for certificate paths to set up a secure server using HTTPS.

// Command struct to hold the JSON payload data
type Command struct {
    Command  string `json:"command"`
    Color    string `json:"color,omitempty"`
    Mode     string `json:"mode,omitempty"`
    Pattern  string `json:"pattern,omitempty"`
    Interval int    `json:"interval,omitempty"`
    Angle    int    `json:"angle,omitempty"` // Add Angle for servo control
}

// Function to execute LED commands
func executeLEDCommand(cmd Command) {
    // Simulate the LED control command with a Python script
    cmdArgs := []string{"-c", fmt.Sprintf("python3 led_control.py %s %s %s %d", cmd.Color, cmd.Mode, cmd.Pattern, cmd.Interval)}
    exec.Command("sh", cmdArgs...).Run()
}

// Function to execute Servo commands
func executeServoCommand(cmd Command) {
    // Determine the servo type based on the command
    var servoType string
    if cmd.Command == "servo-horizontal" {
        servoType = "0"
    } else if cmd.Command == "servo-vertical" {
        servoType = "1"
    } else {
        fmt.Println("Unknown servo command:", cmd.Command)
        return
    }

    // Execute the Python script to control the servo
    cmdArgs := []string{"-c", fmt.Sprintf("python3 servo_control.py %s %d", servoType, cmd.Angle)}
    output, err := exec.Command("sh", cmdArgs...).CombinedOutput()
    if err != nil {
        fmt.Printf("Error executing servo command: %s\n", err)
    }
    fmt.Printf("Python script output: %s\n", output)
}

// CORS Middleware to handle cross-origin requests
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

// Handler function to process commands
func handleCommand(w http.ResponseWriter, r *http.Request) {
    var body bytes.Buffer
    _, err := body.ReadFrom(r.Body)
    if err != nil {
        fmt.Println("Error reading body:", err)
        http.Error(w, "Error reading body", http.StatusBadRequest)
        return
    }
    fmt.Printf("Received body: %s\n", body.String())
    r.Body = ioutil.NopCloser(&body)

    var cmd Command
    decoder := json.NewDecoder(r.Body)
    err = decoder.Decode(&cmd)
    if err != nil {
        fmt.Println("Error decoding JSON:", err)
        http.Error(w, "Error decoding JSON", http.StatusBadRequest)
        return
    }

    fmt.Printf("Command received: %s\n", cmd.Command)
    switch cmd.Command {
    case "set-led":
        executeLEDCommand(cmd)
    case "servo-horizontal", "servo-vertical":
        fmt.Printf("Executing command: %s with angle: %d\n", cmd.Command, cmd.Angle)
        executeServoCommand(cmd)
    case "move-up", "move-down", "move-left", "move-right":
        // Handle movement commands here if needed
        fmt.Printf("Movement command: %s\n", cmd.Command)
    case "increase-speed", "decrease-speed", "honk":
        // Handle other commands here if needed
        fmt.Printf("Other command: %s\n", cmd.Command)
    default:
        fmt.Printf("Unknown command: %s\n", cmd.Command)
    }

    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}

func main() {
    // Load environment variables from .env file
    if err := godotenv.Load("../../config/.env"); err != nil {
        fmt.Println("Error loading .env file:", err)
    }

    // Get certificate and key paths from environment variables
    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    if certPath == "" || keyPath == "" {
        fmt.Println("CERT_PATH or KEY_PATH environment variable is not set")
        return
    }

    fmt.Printf("CERT_PATH: %s\n", certPath)
    fmt.Printf("KEY_PATH: %s\n", keyPath)

    // Setup HTTP server with CORS middleware
    mux := http.NewServeMux()
    mux.HandleFunc("/command", handleCommand)

    server := &http.Server{
        Addr:    ":8080",
        Handler: corsMiddleware(mux),
        TLSConfig: &tls.Config{
            MinVersion: tls.VersionTLS12,
        },
    }

    // Start the secure server
    fmt.Println("Starting secure server on :8080")
    if err := server.ListenAndServeTLS(certPath, keyPath); err != nil {
        fmt.Println("Error starting server:", err)
    }
}
