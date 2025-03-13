// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_real.go

package gpio

import "github.com/stianeikeland/go-rpio/v4"

// RealGPIO implements the GPIO interface for actual Raspberry Pi hardware.
type RealGPIO struct{}

func (r RealGPIO) Open() error {
    return rpio.Open()
}

func (r RealGPIO) Close() error {
    return rpio.Close()
}

func (r RealGPIO) Pin(pin int) GPIOPin {
    return &RealGPIOPin{rpio.Pin(pin)} // ✅ Fixed RealGPIOPin reference
}

// RealGPIOPin wraps the rpio.Pin type to implement the GPIOPin interface.
type RealGPIOPin struct {
    pin rpio.Pin
}

func (p *RealGPIOPin) Input() {
    p.pin.Input()
}

func (p *RealGPIOPin) Output() {
    p.pin.Output()
}

func (p *RealGPIOPin) Read() rpio.State {
    return p.pin.Read()
}

func (p *RealGPIOPin) High() {
    p.pin.High()
}

func (p *RealGPIOPin) Low() {
    p.pin.Low()
}

func (p *RealGPIOPin) SetState(state rpio.State) { // ✅ Implemented SetState
    if state == High {
        p.pin.High()
    } else {
        p.pin.Low()
    }
}
