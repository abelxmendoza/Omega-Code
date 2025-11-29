// File: /Omega-Code/servers/robot_controller_backend/gpio/common.go
package gpio

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
    Read() bool  // ✅ CHANGED FROM `rpio.State` to `bool`
    High()
    Low()
}
