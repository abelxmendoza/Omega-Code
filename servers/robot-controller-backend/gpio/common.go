// File: /Omega-Code/servers/robot-controller-backend/gpio/common.go

/*
Package gpio provides common interfaces and types for GPIO operations.
It includes interfaces for GPIO and GPIO pins, and common constants.
*/

package gpio

import "github.com/stianeikeland/go-rpio/v4"

// GPIO interface defines methods for GPIO operations.
type GPIO interface {
    Open() error
    Close() error
    Pin(int) GPIOPin
}

// GPIOPin interface defines methods for GPIO pin operations.
type GPIOPin interface {
    Input()
    Output()
    Read() rpio.State
    High()
    Low()
    SetState(state rpio.State) // Allows setting a specific state (useful for mocks)
}

// Ensure the correct Low and High states are used from rpio
const (
    Low  = rpio.Low  // GPIO Low state
    High = rpio.High // GPIO High state
)
