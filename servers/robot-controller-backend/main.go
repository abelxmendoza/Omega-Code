package main

import (
    "bytes"
    "crypto/tls"
    "encoding/json"
    "fmt"
    "io/ioutil"
    "net/http"
    "os"

    "github.com/joho/godotenv"
)

type Command struct {
    Command string `json:"command"`
}

// CORS Middleware
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
    fmt.Fprintf(w, "Command executed: %s", cmd.Command)
}

func main() {
    // Load environment variables from .env file
    if err := godotenv.Load("../../config/.env"); err != nil {
        fmt.Println("Error loading .env file:", err)
    }

    certPath := os.Getenv("CERT_PATH")
    keyPath := os.Getenv("KEY_PATH")

    if certPath == "" || keyPath == "" {
        fmt.Println("CERT_PATH or KEY_PATH environment variable is not set")
        return
    }

    fmt.Printf("CERT_PATH: %s\n", certPath)
    fmt.Printf("KEY_PATH: %s\n", keyPath)

    mux := http.NewServeMux()
    mux.HandleFunc("/command", handleCommand)

    server := &http.Server{
        Addr:    ":8080",
        Handler: corsMiddleware(mux),
        TLSConfig: &tls.Config{
            MinVersion: tls.VersionTLS12,
        },
    }

    fmt.Println("Starting secure server on :8080")
    if err := server.ListenAndServeTLS(certPath, keyPath); err != nil {
        fmt.Println("Error starting server:", err)
    }
}
