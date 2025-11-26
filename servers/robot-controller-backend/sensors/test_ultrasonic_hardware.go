// File: test_ultrasonic_hardware.go
// Quick hardware diagnostic for HC-SR04 ultrasonic sensor
// Run: go run test_ultrasonic_hardware.go

package main

import (
	"fmt"
	"log"
	"time"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/host/v3"
	"periph.io/x/host/v3/rpi"
)

func main() {
	log.Println("🔍 Ultrasonic Sensor Hardware Diagnostic")
	log.Println("========================================")

	// Initialize periph
	if _, err := host.Init(); err != nil {
		log.Fatalf("❌ Failed to initialize periph: %v", err)
	}
	log.Println("✅ periph.io initialized")

	// Setup pins
	trigger := rpi.P1_13 // GPIO27 (Physical Pin 13)
	echo := rpi.P1_15    // GPIO22 (Physical Pin 15)

	log.Println("\n📌 Pin Configuration:")
	log.Printf("   Trigger: GPIO27 (Physical Pin 13)")
	log.Printf("   Echo:    GPIO22 (Physical Pin 15)")

	// Configure trigger as output
	if err := trigger.Out(gpio.Low); err != nil {
		log.Fatalf("❌ Failed to configure trigger pin: %v", err)
	}
	log.Println("✅ Trigger pin configured as output")

	// Configure echo as input
	if err := echo.In(gpio.PullNoChange, gpio.NoEdge); err != nil {
		log.Fatalf("❌ Failed to configure echo pin: %v", err)
	}
	log.Println("✅ Echo pin configured as input")

	// Test 1: Check initial echo state
	log.Println("\n🔬 Test 1: Initial Echo Pin State")
	initialState := echo.Read()
	log.Printf("   Echo pin state: %v", initialState)
	if initialState == gpio.High {
		log.Println("   ⚠️  Echo pin is HIGH - this might indicate a wiring issue")
	} else {
		log.Println("   ✅ Echo pin is LOW (expected)")
	}

	// Test 2: Send trigger pulse and check echo response
	log.Println("\n🔬 Test 2: Trigger Pulse and Echo Response")
	log.Println("   Sending trigger pulse...")

	// Send trigger pulse
	trigger.Out(gpio.Low)
	time.Sleep(2 * time.Microsecond)
	trigger.Out(gpio.High)
	time.Sleep(20 * time.Microsecond)
	trigger.Out(gpio.Low)

	log.Println("   Trigger pulse sent (20µs HIGH)")

	// Check echo response
	startTime := time.Now()
	timeout := 100 * time.Millisecond
	echoWentHigh := false

	for time.Since(startTime) < timeout {
		if echo.Read() == gpio.High {
			echoWentHigh = true
			log.Println("   ✅ Echo pin went HIGH (sensor responding!)")
			break
		}
		time.Sleep(100 * time.Microsecond)
	}

	if !echoWentHigh {
		log.Println("   ❌ Echo pin did NOT go HIGH")
		log.Println("\n🔧 Troubleshooting Steps:")
		log.Println("   1. Check power: Sensor needs 5V (VCC) and GND")
		log.Println("   2. Check wiring:")
		log.Println("      - Trigger → GPIO27 (Pin 13)")
		log.Println("      - Echo    → GPIO22 (Pin 15)")
		log.Println("      - VCC     → 5V")
		log.Println("      - GND     → GND")
		log.Println("   3. Try a different sensor (might be faulty)")
		log.Println("   4. Check GPIO permissions: sudo usermod -a -G gpio $USER")
		log.Println("   5. Verify pins with: gpio readall (if installed)")
		return
	}

	// Test 3: Measure echo pulse duration
	log.Println("\n🔬 Test 3: Measuring Echo Pulse Duration")
	pulseStart := time.Now()
	pulseEnd := pulseStart
	timeout = 50 * time.Millisecond // Max ~8.5 meters

	for time.Since(pulseStart) < timeout {
		if echo.Read() == gpio.Low {
			pulseEnd = time.Now()
			break
		}
		time.Sleep(10 * time.Microsecond)
	}

	if pulseEnd == pulseStart {
		log.Println("   ⚠️  Echo pulse did not return LOW (might be out of range)")
	} else {
		duration := pulseEnd.Sub(pulseStart)
		distanceCM := float64(duration.Microseconds()) / 58.0
		log.Printf("   ✅ Echo pulse duration: %v", duration)
		log.Printf("   📏 Calculated distance: %.2f cm (%.2f m)", distanceCM, distanceCM/100.0)

		if distanceCM < 2 || distanceCM > 400 {
			log.Println("   ⚠️  Distance out of valid range (2-400cm)")
		}
	}

	// Test 4: Continuous readings
	log.Println("\n🔬 Test 4: Continuous Readings (5 attempts)")
	successCount := 0
	for i := 0; i < 5; i++ {
		trigger.Out(gpio.Low)
		time.Sleep(2 * time.Microsecond)
		trigger.Out(gpio.High)
		time.Sleep(20 * time.Microsecond)
		trigger.Out(gpio.Low)

		startWait := time.Now()
		for echo.Read() == gpio.Low {
			if time.Since(startWait) > 100*time.Millisecond {
				log.Printf("   Attempt %d: ❌ Timeout", i+1)
				break
			}
		}

		if echo.Read() == gpio.High {
			pulseStart := time.Now()
			for echo.Read() == gpio.High {
				if time.Since(pulseStart) > 50*time.Millisecond {
					break
				}
			}
			duration := time.Since(pulseStart)
			distanceCM := float64(duration.Microseconds()) / 58.0
			if distanceCM >= 2 && distanceCM <= 400 {
				log.Printf("   Attempt %d: ✅ %.1f cm", i+1, distanceCM)
				successCount++
			} else {
				log.Printf("   Attempt %d: ⚠️  Invalid reading (%.1f cm)", i+1, distanceCM)
			}
		}

		time.Sleep(100 * time.Millisecond)
	}

	log.Println("\n📊 Summary:")
	log.Printf("   Successful readings: %d/5", successCount)
	if successCount == 0 {
		log.Println("\n❌ Sensor is not responding. Check hardware connections.")
	} else if successCount < 3 {
		log.Println("\n⚠️  Sensor is responding but inconsistently. Check wiring and power.")
	} else {
		log.Println("\n✅ Sensor is working correctly!")
	}
}

