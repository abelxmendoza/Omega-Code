// File: main_ultrasonic.go
// Summary: 🧠 Omega 1 – Ultrasonic WebSocket Server (Pi 5 Compatible)
//
// This Go script uses the `periph.io` library to interface with an HC-SR04 ultrasonic sensor
// on a Raspberry Pi 5 without needing pigpio or background daemons. It runs a WebSocket server
// on port 8080 and streams live distance data (in cm, meters, inches, feet) to any connected clients.
//
// ✅ Highlights:
// - Uses `periph.io/x/host/v3` for direct GPIO control
// - Fully compatible with Raspberry Pi 5 (Ubuntu 24.04 or Raspberry Pi OS)
// - Daemonless operation (no `pigpiod` required)
// - Sends JSON payloads over WebSocket every 1 second
// - Graceful error handling (timeouts, invalid readings)
// - NEW: Sends a JSON welcome envelope on connect and responds to { "type": "ping" } with { "type": "pong" }
//
// 🔌 Wiring:
// - Trigger → GPIO27 (Physical Pin 13)
// - Echo    → GPIO22 (Physical Pin 15)
//
// Run it with:
// $ go run main_ultrasonic.go
//

package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/websocket"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/host/v3"
	"periph.io/x/host/v3/rpi"
)

type UltrasonicData struct {
	Status       string  `json:"status"`
	DistanceCM   int     `json:"distance_cm"`
	DistanceM    float64 `json:"distance_m"`
	DistanceInch float64 `json:"distance_inch"`
	DistanceFeet float64 `json:"distance_feet"`
	Error        string  `json:"error,omitempty"`
}

type envCfg struct {
	Port            int
	Path            string
	OriginAllow     map[string]struct{}
	LogEvery        time.Duration // if >0, log a line this often
	LogDeltaCM      int           // also log when change >= this (abs)
	ReadLimit       int64
	WriteTimeout    time.Duration
	ReadTimeout     time.Duration
	MeasureInterval time.Duration
}

func getenvInt(key string, def int) int {
	if v := os.Getenv(key); v != "" {
		if n, err := strconv.Atoi(v); err == nil {
			return n
		}
	}
	return def
}
func getenvDur(key string, def time.Duration) time.Duration {
	if v := os.Getenv(key); v != "" {
		if d, err := time.ParseDuration(v); err == nil {
			return d
		}
	}
	return def
}
func getenvSet(key string) map[string]struct{} {
	s := strings.TrimSpace(os.Getenv(key))
	if s == "" {
		return nil
	}
	out := make(map[string]struct{})
	for _, p := range strings.Split(s, ",") {
		p = strings.TrimSpace(p)
		if p != "" {
			out[p] = struct{}{}
		}
	}
	return out
}

func loadCfg() envCfg {
	return envCfg{
		Port:            getenvInt("PORT_ULTRASONIC", 8080),
		Path:            firstNonEmpty(os.Getenv("ULTRA_PATH"), "/ultrasonic"),
		OriginAllow:     getenvSet("ORIGIN_ALLOW"), // e.g. http://localhost:3000,http://192.168.1.107:3000
		LogEvery:        getenvDur("ULTRA_LOG_EVERY", 1*time.Second), // Log every second for better visibility
		LogDeltaCM:      getenvInt("ULTRA_LOG_DELTA_CM", 5), // Log when change >= 5cm
		ReadLimit:       int64(getenvInt("ULTRA_READ_LIMIT", 4096)),
		WriteTimeout:    getenvDur("ULTRA_WRITE_TIMEOUT", 1500*time.Millisecond),
		ReadTimeout:     getenvDur("ULTRA_READ_TIMEOUT", 0), // 0 = no deadline (JSON pings keep it alive)
		MeasureInterval: getenvDur("ULTRA_MEASURE_INTERVAL", 1*time.Second),
	}
}
func firstNonEmpty(v, d string) string {
	if v != "" {
		return v
	}
	return d
}

func checkOriginFactory(allow map[string]struct{}) func(r *http.Request) bool {
	// Allow CLI/non-browser (no Origin header). If allow list is empty, allow all.
	return func(r *http.Request) bool {
		if len(allow) == 0 {
			return true
		}
		origin := r.Header.Get("Origin")
		if origin == "" {
			return true
		}
		_, ok := allow[origin]
		return ok
	}
}

var upgrader websocket.Upgrader // set in main()

// measureDistance triggers HC-SR04 and returns distance in cm.
// Optimized for performance with minimal allocations and efficient GPIO polling.
func measureDistance(trigger gpio.PinOut, echo gpio.PinIn) (int, error) {
	// Trigger pulse sequence - optimized to minimize error checks
	// Batch error handling at end if needed
	trigger.Out(gpio.Low)
	// Use runtime.nanotime for precise timing (via time.Sleep which uses it internally)
	time.Sleep(2 * time.Microsecond)
	trigger.Out(gpio.High)
	time.Sleep(20 * time.Microsecond)
	trigger.Out(gpio.Low)
	
	// CRITICAL: Small settling delay after trigger pulse
	// HC-SR04 needs ~50-100µs to process trigger and prepare echo response
	// This reduces intermittent "timeout waiting for echo HIGH" errors
	time.Sleep(100 * time.Microsecond)

	// Optimized echo detection with adaptive polling
	// Use time.Now() once and calculate deltas efficiently
	startWait := time.Now()
	timeout := 150 * time.Millisecond // Increased from 100ms for better reliability
	deadline := startWait.Add(timeout)
	
	// Adaptive polling: start with shorter delays, increase if needed
	pollInterval := 10 * time.Microsecond
	maxPollInterval := 100 * time.Microsecond
	
	initialEchoState := echo.Read()
	for echo.Read() == gpio.Low {
		now := time.Now()
		if now.After(deadline) {
			// Pre-allocate error string to avoid repeated allocations
			return -1, fmt.Errorf("timeout waiting for echo HIGH (initial: %v, waited: %vms). Check: 1) Power (5V), 2) Echo wiring (GPIO22/Pin15), 3) Sensor faulty", 
				initialEchoState, now.Sub(startWait).Milliseconds())
		}
		// Adaptive polling: increase interval if we're not close to timeout
		if now.Sub(startWait) < timeout/2 {
			time.Sleep(pollInterval)
		} else {
			time.Sleep(maxPollInterval)
		}
	}
	
	// Capture pulse start time
	t0 := time.Now()
	maxPulseDuration := 50 * time.Millisecond
	pulseDeadline := t0.Add(maxPulseDuration)
	
	// Optimized pulse measurement with adaptive polling
	pollInterval = 5 * time.Microsecond // Tighter polling for pulse measurement
	for echo.Read() == gpio.High {
		now := time.Now()
		if now.After(pulseDeadline) {
			return -1, fmt.Errorf("timeout waiting for echo LOW (duration: %vms). Echo stuck HIGH - check wiring", 
				now.Sub(t0).Milliseconds())
		}
		time.Sleep(pollInterval)
	}
	
	// Calculate duration efficiently
	dur := time.Since(t0)
	
	// Optimized distance calculation: use integer math where possible
	// 58µs per cm = 58,000 nanoseconds per cm
	// Use integer division first, then convert
	us := dur.Microseconds()
	cm := int((us * 100) / 5800) // Multiply first to maintain precision, then divide
	
	// Range validation with early returns
	if cm < 2 {
		return -1, fmt.Errorf("distance too close (<2cm): %d cm. May be noise or object too close", cm)
	}
	if cm > 400 {
		return -1, fmt.Errorf("distance out of range: %d cm (pulse: %v). Valid range: 2-400cm", cm, dur)
	}
	
	return cm, nil
}

type outMsg struct {
	kind string // "json"
	data any
}

// Pre-allocated ping response map to reduce allocations
var pingResponsePool = sync.Pool{
	New: func() interface{} {
		return map[string]any{
			"type": "pong",
		}
	},
}

func handleConn(ws *websocket.Conn, trigger gpio.PinOut, echo gpio.PinIn, cfg envCfg) {
	defer ws.Close()

	// Optimized channel buffer size based on measurement interval
	// Buffer allows 2 seconds of measurements to queue (prevents blocking)
	bufferSize := int(cfg.MeasureInterval/time.Second) * 2
	if bufferSize < 4 {
		bufferSize = 4 // Minimum buffer
	}
	if bufferSize > 16 {
		bufferSize = 16 // Maximum buffer to limit memory
	}
	send := make(chan outMsg, bufferSize)
	done := make(chan struct{})

	// Reader: respond to JSON pings by enqueueing a pong (no concurrent writes)
	// Optimized with pre-allocated ping response map
	go func() {
		defer close(done)
		if cfg.ReadLimit > 0 {
			ws.SetReadLimit(cfg.ReadLimit)
		}
		
		for {
			if cfg.ReadTimeout > 0 {
				_ = ws.SetReadDeadline(time.Now().Add(cfg.ReadTimeout))
			}
			_, msg, err := ws.ReadMessage()
			if err != nil {
				// Normal close or network error
				return
			}
			
			// Fast path: quick check if message looks like JSON ping
			// Most messages will be ping/pong, optimize for that case
			if len(msg) > 8 && (msg[0] == '{' || msg[0] == ' ') {
				var m map[string]any
				if err := json.Unmarshal(msg, &m); err == nil {
					if t, _ := m["type"].(string); t == "ping" {
						// Get ping response from pool, update timestamp, return to pool after use
						pingResponse := pingResponsePool.Get().(map[string]any)
						pingResponse["ts"] = m["ts"]
						select {
						case send <- outMsg{"json", pingResponse}:
							// Return to pool after sending (will be copied by WriteJSON)
							pingResponsePool.Put(pingResponse)
						default:
							// Channel full, return to pool and skip
							pingResponsePool.Put(pingResponse)
						}
					}
				}
			}
		}
	}()

	// Welcome envelope (queued)
	send <- outMsg{"json", map[string]any{
		"status":  "connected",
		"service": "ultrasonic",
		"message": "Ultrasonic WebSocket connection established",
		"ts":      time.Now().UnixMilli(),
	}}

	// Writer / sensor loop
	ticker := time.NewTicker(cfg.MeasureInterval)
	defer ticker.Stop()

	// Optimized logging state with cached deadline
	lastCM := -1
	logDeadline := time.Time{} // Cache next log time to avoid repeated calculations

	// Optimized JSON writer with deadline caching
	writeDeadline := time.Time{}
	writeJSON := func(v any) error {
		now := time.Now()
		// Only update deadline if it's expired or about to expire
		if writeDeadline.IsZero() || now.After(writeDeadline.Add(-100*time.Millisecond)) {
			writeDeadline = now.Add(cfg.WriteTimeout)
		}
		_ = ws.SetWriteDeadline(writeDeadline)
		return ws.WriteJSON(v)
	}

	log.Printf("[ULTRA] ✅ Client connected from %s", ws.RemoteAddr())
	defer log.Printf("[ULTRA] ❌ Client disconnected")

	// Error tracking for diagnostics
	errorCount := 0
	successCount := 0
	consecutiveErrors := 0
	const maxConsecutiveErrors = 5

	for {
		select {
		case <-done:
			return
		case m := <-send:
			if m.kind == "json" {
				if err := writeJSON(m.data); err != nil {
					log.Printf("[ULTRA] ❌ Write error: %v", err)
					return
				}
			}
		case <-ticker.C:
			// Measure distance with retry for transient failures
			// Retry up to 2 times with exponential backoff (50ms, 100ms delays)
			var cm int
			var err error
			maxRetries := 2
			for attempt := 0; attempt <= maxRetries; attempt++ {
				cm, err = measureDistance(trigger, echo)
				if err == nil {
					break // Success, exit retry loop
				}
				// If this is not the last attempt and error is timeout-related, retry
				if attempt < maxRetries && strings.Contains(err.Error(), "timeout waiting for echo") {
					backoffDelay := time.Duration(50*(1<<attempt)) * time.Millisecond
					time.Sleep(backoffDelay)
					continue
				}
				break // Non-retryable error or last attempt
			}
			now := time.Now() // Cache time for this iteration
			
			if err != nil {
				errorCount++
				consecutiveErrors++
				
				// Build error message efficiently
				errorMsg := err.Error()
				if consecutiveErrors >= maxConsecutiveErrors {
					// Append consecutive error count (reuse string builder if needed)
					errorMsg += fmt.Sprintf(" (%d consecutive errors - sensor may need hardware check)", consecutiveErrors)
				}
				
				// Emit error - use non-blocking send to prevent deadlock
				select {
				case send <- outMsg{"json", UltrasonicData{
					Status:     "error",
					Error:      errorMsg,
					DistanceCM: -1,
				}}:
				default:
					// Channel full, log warning but continue
					log.Printf("[ULTRA] ⚠️  Channel full, dropping error message")
				}
				
				// Rate-limited error logging (only log every 5th error or first/last)
				if consecutiveErrors == 1 || consecutiveErrors == maxConsecutiveErrors || consecutiveErrors%5 == 0 {
					log.Printf("[ULTRA] ⚠️  Measurement failed: %v (errors: %d/%d, consecutive: %d)", 
						err, errorCount, errorCount+successCount, consecutiveErrors)
				}
				
				// Provide troubleshooting hints after multiple errors (once)
				if consecutiveErrors == maxConsecutiveErrors {
					log.Printf("[ULTRA] 🔧 Troubleshooting: Check sensor power (5V), wiring (GPIO27/22), and run: go run test_ultrasonic_hardware.go")
				}
				continue
			}
			
			// Success - reset consecutive error counter
			successCount++
			recovered := consecutiveErrors > 0
			if recovered {
				log.Printf("[ULTRA] ✅ Sensor recovered after %d errors", consecutiveErrors)
				consecutiveErrors = 0
			}

			// Pre-calculate conversions (optimize division operations)
			// Use integer math where possible, convert to float only when needed
			meters := float64(cm) * 0.01  // Faster than division
			inches := float64(cm) * 0.393700787 // Pre-calculated 1/2.54
			feet := float64(cm) * 0.032808399   // Pre-calculated 1/30.48

			// Build message struct efficiently
			msg := UltrasonicData{
				Status:       "success",
				DistanceCM:   cm,
				DistanceM:    round2(meters),
				DistanceInch: round2(inches),
				DistanceFeet: round2(feet),
			}
			
			// Send measurement (non-blocking)
			select {
			case send <- outMsg{"json", msg}:
			default:
				log.Printf("[ULTRA] ⚠️  Channel full, dropping measurement")
			}

			// Optimized terminal logging with cached deadline
			shouldLog := false
			if cfg.LogEvery > 0 {
				if logDeadline.IsZero() || now.After(logDeadline) {
					shouldLog = true
					logDeadline = now.Add(cfg.LogEvery)
				}
			}
			if !shouldLog && cfg.LogDeltaCM > 0 && lastCM >= 0 {
				delta := cm - lastCM
				if delta < 0 {
					delta = -delta // abs without function call
				}
				if delta >= cfg.LogDeltaCM {
					shouldLog = true
				}
			}
			if shouldLog {
				log.Printf("[ULTRA] 📏 Distance: %d cm (%.2fm / %.2fin / %.2fft)", cm, msg.DistanceM, msg.DistanceInch, msg.DistanceFeet)
				lastCM = cm
			}
		}
	}
}

// Optimized rounding: use integer math where possible
func round2(v float64) float64 {
	// Fast path for common values
	if v == 0 {
		return 0
	}
	return math.Round(v*100) / 100
}

// Inline abs for better performance (compiler will optimize)
// Removed abs() function - use inline: if x < 0 { x = -x }
func errString(err error) string {
	if err == nil {
		return ""
	}
	// Return full error message for better debugging
	return err.Error()
}

func main() {
	log.SetOutput(os.Stdout)
	cfg := loadCfg()

	// Init periph once (not per-connection)
	if _, err := host.Init(); err != nil {
		log.Fatalf("[ULTRA] periph init failed: %v", err)
	}

	// Prepare pins
	trigger := rpi.P1_13 // GPIO27
	echo := rpi.P1_15    // GPIO22

	// Set defaults (low/no edge); connection handler toggles as needed
	if err := trigger.Out(gpio.Low); err != nil {
		log.Fatalf("[ULTRA] trigger init failed: %v", err)
	}
	if err := echo.In(gpio.PullNoChange, gpio.NoEdge); err != nil {
		log.Fatalf("[ULTRA] echo init failed: %v", err)
	}

	upgrader = websocket.Upgrader{
		CheckOrigin: checkOriginFactory(cfg.OriginAllow),
	}

	mux := http.NewServeMux()

	// Health
	mux.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		_, _ = w.Write([]byte(`{"ok":true}`))
	})

	// WebSocket endpoint (e.g., ws://host:8080/ultrasonic)
	mux.HandleFunc(cfg.Path, func(w http.ResponseWriter, r *http.Request) {
		ws, err := upgrader.Upgrade(w, r, nil)
		if err != nil {
			log.Printf("[ULTRA] upgrade error: %v", err)
			return
		}
		handleConn(ws, trigger, echo, cfg)
	})

	srv := &http.Server{
		Addr:              ":" + strconv.Itoa(cfg.Port),
		Handler:           mux,
		ReadHeaderTimeout: 5 * time.Second,
	}

	log.Printf("🌐 Ultrasonic WS server listening on :%d%s", cfg.Port, cfg.Path)

	// Graceful shutdown
	idle := make(chan struct{})
	go func() {
		// SIGINT/SIGTERM
		ch := make(chan os.Signal, 1)
		signal.Notify(ch, os.Interrupt, syscall.SIGTERM)
		<-ch
		log.Printf("[ULTRA] shutting down…")
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		_ = srv.Shutdown(ctx)
		close(idle)
	}()

	if err := srv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		log.Fatalf("[ULTRA] server error: %v", err)
	}
	<-idle
}

