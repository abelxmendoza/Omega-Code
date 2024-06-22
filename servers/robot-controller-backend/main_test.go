// File: /Omega-Code/servers/robot-controller-backend/main_test.go

package main

import (
    "bytes"
    "encoding/json"
    "net/http"
    "net/http/httptest"
    "testing"
)

func TestHandleCommand(t *testing.T) {
    server := httptest.NewServer(http.HandlerFunc(handleCommand))
    defer server.Close()

    command := Command{
        Command:   "servo-horizontal",
        Angle:     10,
        RequestID: "test-request-id",
    }

    jsonCommand, _ := json.Marshal(command)
    resp, err := http.Post(server.URL, "application/json", bytes.NewBuffer(jsonCommand))
    if err != nil {
        t.Fatalf("Failed to send POST request: %v", err)
    }

    if resp.StatusCode != http.StatusOK {
        t.Errorf("Expected status code %d, got %d", http.StatusOK, resp.StatusCode)
    }
}

func TestUnknownCommand(t *testing.T) {
    server := httptest.NewServer(http.HandlerFunc(handleCommand))
    defer server.Close()

    command := Command{
        Command:   "unknown-command",
        Angle:     10,
        RequestID: "test-request-id",
    }

    jsonCommand, _ := json.Marshal(command)
    resp, err := http.Post(server.URL, "application/json", bytes.NewBuffer(jsonCommand))
    if err != nil {
        t.Fatalf("Failed to send POST request: %v", err)
    }

    if resp.StatusCode != http.StatusBadRequest {
        t.Errorf("Expected status code %d, got %d", http.StatusBadRequest, resp.StatusCode)
    }
}
