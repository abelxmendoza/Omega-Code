/*
# File: /Omega-Code/servers/robot-controller-backend/sensors/main_ultrasonic.go

Ultrasonic Sensor Control and Data Processing with WebSockets

This script handles the ultrasonic sensor functionality for a Raspberry Pi,
including reading sensor data, processing it using Rust integration, and
sending the data through WebSockets.
*/

package main

import (
	"encoding/json"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/gorilla/websocket"
	"github.com/stianeikeland/go-rpio"
)

/*
#cgo LDFLAGS: -L/home/omega1/Documents/code/Omega-Code/servers/robot-controller-backend/rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_ultrasonic_data(char* input);
*/
import "C"
import (
	"fmt"
	"unsafe"
)

type UltrasonicData struct {
	Distance int `json:"distance"`
}

func processUltrasonicData(input string) string {
	cInput := C.CString(input)
	defer C.free(unsafe.Pointer(cInput))

	cOutput := C.process_ultrasonic_data(cInput)
	defer C.free(unsafe.Pointer(cOutput))

	return C.GoString(cOutput)
}

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin: func(r *http.Request) bool {
		return true // Adjust this according to your CORS policy
	},
}

func handleUltrasonicSensor(ws *websocket.Conn) {
	if err := rpio.Open(); err != nil {
		log.Printf("Error opening GPIO: %s\n", err)
		return
	}
	defer rpio.Close()

	trigger := rpio.Pin(27)
	echo := rpio.Pin(22)

	trigger.Output()
	echo.Input()

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		trigger.Low()
		time.Sleep(2 * time.Microsecond)
		trigger.High()
		time.Sleep(10 * time.Microsecond)
		trigger.Low()

		start := time.Now()
		for echo.Read() == rpio.Low {
		}
		start = time.Now()

		for echo.Read() == rpio.High {
		}
		duration := time.Since(start)
		distance := int(duration.Seconds() * 17150)

		data := UltrasonicData{Distance: distance}
		log.Printf("Distance: %d cm", data.Distance)

		err := ws.WriteJSON(data)
		if err != nil {
			log.Printf("WriteJSON error: %v", err)
			return
		}
	}
}

func main() {
	log.SetOutput(os.Stdout)
	log.Printf("Starting ultrasonic sensor WebSocket server")

	http.HandleFunc("/ultrasonic", func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("Upgrade error: %v", err)
			return
		}
		defer ws.Close()

		handleUltrasonicSensor(ws)
	})

	addr := ":8080"
	log.Printf("Starting server on %s", addr)
	log.Fatal(http.ListenAndServe(addr, nil))
}
