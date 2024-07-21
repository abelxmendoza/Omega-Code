// File: /Omega-Code/servers/robot-controller-backend/main_combined.go

package main

import (
    "log"
    "net/http"
    "os"

    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/commands"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/common"
)

func main() {
    log.SetOutput(os.Stdout)
    log.Printf("Starting server setup...")

    commands.InitGPIO()

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

    go commands.StartROSNodes()
    go commands.StartPythonVideoServer()

    mux := http.NewServeMux()
    mux.HandleFunc("/command", commands.HandleCommand)
    mux.HandleFunc("/ws", commands.HandleConnections)

    server := &http.Server{
        Addr:    ":8080",
        Handler: common.CorsMiddleware(mux),
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

