// File: /Omega-Code/servers/robot-controller-backend/gpio/common.go

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
    SetState(state rpio.State) // ✅ Added SetState method
}

var (
    Low  rpio.State = rpio.Low
    High rpio.State = rpio.High
)
