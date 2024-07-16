package main

import (
    "bytes"
    "encoding/json"
    "net/http"
    "net/http/httptest"
    "os"
    "os/exec"
    "testing"
)

// Mock for exec.Command
func mockCommand(command string, args ...string) *exec.Cmd {
    cmd := exec.Command("echo", "Mock command executed")
    return cmd
}

// Override execCommand for tests
func TestMain(m *testing.M) {
    execCommand = mockCommand
    gpio = &MockGPIO{} // Inject the mock implementation
    code := m.Run()
    os.Exit(code)
}

func TestHandleCommand(t *testing.T) {
    reqBody := bytes.NewBufferString(`{"command":"servo-horizontal","angle":90,"request_id":"1"}`)
    req, err := http.NewRequest("POST", "/command", reqBody)
    if err != nil {
        t.Fatal(err)
    }

    rr := httptest.NewRecorder()
    handler := http.HandlerFunc(handleCommand)

    handler.ServeHTTP(rr, req)

    if status := rr.Code; status != http.StatusOK {
        t.Errorf("handler returned wrong status code: got %v want %v", status, http.StatusOK)
    }

    expected := `Command executed: servo-horizontal`
    if rr.Body.String() != expected {
        t.Errorf("handler returned unexpected body: got %v want %v", rr.Body.String(), expected)
    }
}

func TestHandleLineTracking(t *testing.T) {
    req, err := http.NewRequest("GET", "/line-tracking", nil)
    if err != nil {
        t.Fatal(err)
    }

    rr := httptest.NewRecorder()
    handler := http.HandlerFunc(handleLineTracking)

    handler.ServeHTTP(rr, req)

    if status := rr.Code; status != http.StatusOK {
        t.Errorf("handler returned wrong status code: got %v want %v", status, http.StatusOK)
    }

    var expected = LineTrackingData{IR01: 1, IR02: 0, IR03: 1}
    var got LineTrackingData
    json.NewDecoder(rr.Body).Decode(&got)
    if got != expected {
        t.Errorf("handler returned unexpected body: got %v want %v", got, expected)
    }
}

func TestHandleUltrasonicSensor(t *testing.T) {
    req, err := http.NewRequest("GET", "/ultrasonic-sensor", nil)
    if err != nil {
        t.Fatal(err)
    }

    rr := httptest.NewRecorder()
    handler := http.HandlerFunc(handleUltrasonicSensor)

    handler.ServeHTTP(rr, req)

    if status := rr.Code; status != http.StatusOK {
        t.Errorf("handler returned wrong status code: got %v want %v", status, http.StatusOK)
    }

    var expected = UltrasonicData{Distance: 100}
    var got UltrasonicData
    json.NewDecoder(rr.Body).Decode(&got)
    if got != expected {
        t.Errorf("handler returned unexpected body: got %v want %v", got, expected)
    }
}
