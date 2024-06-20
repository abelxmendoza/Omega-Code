package main

import (
    "encoding/json"
    "fmt"
    "log"
    "net/http"
    "os"

    "github.com/joho/godotenv"
)

type Command struct {
    Command string `json:"command"`
}

func commandHandler(w http.ResponseWriter, r *http.Request) {
    if r.Method == http.MethodOptions {
        w.Header().Set("Access-Control-Allow-Origin", "*")
        w.Header().Set("Access-Control-Allow-Methods", "POST, OPTIONS")
        w.Header().Set("Access-Control-Allow-Headers", "Content-Type")
        w.WriteHeader(http.StatusOK)
        return
    }

    log.Println("Received a request")

    var cmd Command
    err := json.NewDecoder(r.Body).Decode(&cmd)
    if err != nil {
        log.Printf("Error decoding JSON: %v\n", err)
        http.Error(w, "Error decoding JSON", http.StatusBadRequest)
        return
    }

    log.Printf("Command received: %s", cmd.Command)

    switch cmd.Command {
    case "move-up":
        log.Println("Executing command: move-up")
        // Add code to send command to Raspberry Pi to move the robot up
    case "move-left":
        log.Println("Executing command: move-left")
        // Add code to send command to Raspberry Pi to move the robot left
    case "move-down":
        log.Println("Executing command: move-down")
        // Add code to send command to Raspberry Pi to move the robot down
    case "move-right":
        log.Println("Executing command: move-right")
        // Add code to send command to Raspberry Pi to move the robot right
    case "camera-up":
        log.Println("Executing command: camera-up")
        // Add code to send command to Raspberry Pi to move the camera up
    case "camera-left":
        log.Println("Executing command: camera-left")
        // Add code to send command to Raspberry Pi to move the camera left
    case "camera-down":
        log.Println("Executing command: camera-down")
        // Add code to send command to Raspberry Pi to move the camera down
    case "camera-right":
        log.Println("Executing command: camera-right")
        // Add code to send command to Raspberry Pi to move the camera right
    case "increase-speed":
        log.Println("Executing command: increase-speed")
        // Add code to send command to Raspberry Pi to increase speed
    case "decrease-speed":
        log.Println("Executing command: decrease-speed")
        // Add code to send command to Raspberry Pi to decrease speed
    case "stop":
        log.Println("Executing command: stop")
        // Add code to send command to Raspberry Pi to stop
    default:
        log.Printf("Invalid command received: %s", cmd.Command)
        http.Error(w, "Invalid command", http.StatusBadRequest)
        return
    }

    w.Header().Set("Access-Control-Allow-Origin", "*")
    w.WriteHeader(http.StatusOK)
    w.Write([]byte(fmt.Sprintf("Command executed: %s", cmd.Command)))
}

func main() {
    // Load environment variables from .env file
    err := godotenv.Load("config/.env")
    if err != nil {
        log.Fatalf("Error loading .env file")
    }

    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    http.HandleFunc("/command", commandHandler)
    log.Println("Starting server on :8080")

    // Start the server with TLS
    if err := http.ListenAndServeTLS(":8080", certPath, keyPath, nil); err != nil {
        log.Fatalf("Could not start server: %s\n", err.Error())
    }
}

