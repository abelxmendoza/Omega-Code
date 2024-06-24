package main

import (
    "encoding/json"
    "log"
    "net/http"
    "time"
    "github.com/stianeikeland/go-rpio"
)

type LineTrackingData struct {
    IR01 int `json:"ir01"`
    IR02 int `json:"ir02"`
    IR03 int `json:"ir03"`
}

func handleLineTracking(w http.ResponseWriter, r *http.Request) {
    logRequest(r)

    if err := rpio.Open(); err != nil {
        log.Printf("Error opening GPIO: %s\n", err)
        http.Error(w, "Error opening GPIO", http.StatusInternalServerError)
        return
    }
    defer rpio.Close()

    ir01 := rpio.Pin(14)
    ir02 := rpio.Pin(15)
    ir03 := rpio.Pin(23)

    ir01.Input()
    ir02.Input()
    ir03.Input()

    data := LineTrackingData{
        IR01: ir01.Read(),
        IR02: ir02.Read(),
        IR03: ir03.Read(),
    }

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(data)
}
