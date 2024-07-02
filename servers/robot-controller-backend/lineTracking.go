// +build rpi

package main

import (
    "encoding/json"
    "log"
    "net/http"
    "os/exec"
    "github.com/stianeikeland/go-rpio"
)

// Rust integration
/*
#cgo LDFLAGS: -L./rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_line_tracking_data(char* input);
*/
import "C"
import (
    "fmt"
    "unsafe"
)

func processLineTrackingData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_line_tracking_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}

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
        IR01: int(ir01.Read()),
        IR02: int(ir02.Read()),
        IR03: int(ir03.Read()),
    }

    // Process data using Rust
    input := fmt.Sprintf("%d,%d,%d", data.IR01, data.IR02, data.IR03)
    output := processLineTrackingData(input)
    log.Printf("Processed data: %s", output)

    w.Header().Set("Content-Type", "application/json")
    json.NewEncoder(w).Encode(data)

    // Optional: Execute the Python script
    cmd := exec.Command("python3", "line_tracking.py")
    err := cmd.Run()
    if err != nil {
        log.Printf("Error executing line tracking Python script: %s\n", err)
    }
}
