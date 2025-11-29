// File: /Omega-Code/servers/robot_controller_backend/rust_integration/rust_integration.go

/*
Package rust_integration provides functions for integrating Rust modules with Go.
It includes functions for processing data using Rust libraries for ultrasonic sensors and line tracking.

The Rust functions are linked using CGO and are called from Go code to process sensor data.
*/

package rust_integration

/*
#cgo LDFLAGS: -L./rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_ultrasonic_data(char* input);
extern char* process_line_tracking_data(char* input);
*/
import "C"
import (
    "unsafe"
)

// ProcessUltrasonicData processes the input data using the Rust ultrasonic data processing function.
func ProcessUltrasonicData(input string) string {
    cInput := C.CString(input) // Convert Go string to C string
    defer C.free(unsafe.Pointer(cInput)) // Ensure the C string is freed after usage

    cOutput := C.process_ultrasonic_data(cInput) // Call the Rust function
    result := C.GoString(cOutput) // Convert C string to Go string
    C.free(unsafe.Pointer(cOutput)) // Ensure the output C string is freed after conversion

    return result
}

// ProcessLineTrackingData processes the input data using the Rust line tracking data processing function.
func ProcessLineTrackingData(input string) string {
    cInput := C.CString(input) // Convert Go string to C string
    defer C.free(unsafe.Pointer(cInput)) // Ensure the C string is freed after usage

    cOutput := C.process_line_tracking_data(cInput) // Call the Rust function
    result := C.GoString(cOutput) // Convert C string to Go string
    C.free(unsafe.Pointer(cOutput)) // Ensure the output C string is freed after conversion

    return result
}

