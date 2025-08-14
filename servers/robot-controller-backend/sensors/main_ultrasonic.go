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
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
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
		LogEvery:        getenvDur("ULTRA_LOG_EVERY", 5 * time.Second),
		LogDeltaCM:      getenvInt("ULTRA_LOG_DELTA_CM", 10),
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
func measureDistance(trigger gpio.PinOut, echo gpio.PinIn) (int, error) {
	// Ensure low, then 10µs high pulse
	trigger.Out(gpio.Low)
	time.Sleep(2 * time.Microsecond)
	trigger.Out(gpio.High)
	time.Sleep(10 * time.Microsecond)
	trigger.Out(gpio.Low)

	// Wait for echo high (start) with timeout
	startWait := time.Now()
	for echo.Read() == gpio.Low {
		if time.Since(startWait) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	t0 := time.Now()

	// Wait for echo low (end) with timeout
	for echo.Read() == gpio.High {
		if time.Since(t0) > time.Second {
			return -1, os.ErrDeadlineExceeded
		}
	}
	dur := time.Since(t0)

	// Convert to cm (~58µs per cm round-trip)
	cm := int(float64(dur.Microseconds()) / 58.0)
	if cm <= 0 || cm > 400 {
		return -1, os.ErrInvalid
	}
	return cm, nil
}

type outMsg struct {
	kind string // "json"
	data any
}

func handleConn(ws *websocket.Conn, trigger gpio.PinOut, echo gpio.PinIn, cfg envCfg) {
	defer ws.Close()

	// Safety: single writer goroutine, reader posts pongs to 'send'
	send := make(chan outMsg, 8)
	done := make(chan struct{})

	// Reader: respond to JSON pings by enqueueing a pong (no concurrent writes)
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
			var m map[string]any
			if err := json.Unmarshal(msg, &m); err == nil {
				if t, _ := m["type"].(string); t == "ping" {
					send <- outMsg{"json", map[string]any{
						"type": "pong",
						"ts":   m["ts"],
					}}
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

	lastLogTime := time.Time{}
	lastCM := -1

	writeJSON := func(v any) error {
		_ = ws.SetWriteDeadline(time.Now().Add(cfg.WriteTimeout))
		return ws.WriteJSON(v)
	}

	log.Printf("[ULTRA] client connected")
	defer log.Printf("[ULTRA] client disconnected")

	for {
		select {
		case <-done:
			return
		case m := <-send:
			if m.kind == "json" {
				if err := writeJSON(m.data); err != nil {
					return
				}
			}
		case <-ticker.C:
			cm, err := measureDistance(trigger, echo)
			if err != nil {
				// Emit an error sample (UI can display)
				_ = writeJSON(UltrasonicData{Status: "error", Error: errString(err)})
				// Log timeouts sparingly
				if errors.Is(err, os.ErrDeadlineExceeded) {
					// debounce repeated timeouts
					if time.Since(lastLogTime) > 3*time.Second {
						log.Printf("[ULTRA] timeout waiting for echo")
						lastLogTime = time.Now()
					}
				}
				continue
			}

			meters := float64(cm) / 100.0
			inches := float64(cm) / 2.54
			feet := float64(cm) / 30.48

			msg := UltrasonicData{
				Status:       "success",
				DistanceCM:   cm,
				DistanceM:    round2(meters),
				DistanceInch: round2(inches),
				DistanceFeet: round2(feet),
			}
			if err := writeJSON(msg); err != nil {
				return
			}

			// Optional terminal logging (rate-limited and/or when changed significantly)
			shouldLog := false
			if cfg.LogEvery > 0 {
				if time.Since(lastLogTime) >= cfg.LogEvery {
					shouldLog = true
				}
			}
			if cfg.LogDeltaCM > 0 && lastCM >= 0 && abs(cm-lastCM) >= cfg.LogDeltaCM {
				shouldLog = true
			}
			if shouldLog {
				log.Printf("[ULTRA] %d cm (%.2fm / %.2fin / %.2fft)", cm, msg.DistanceM, msg.DistanceInch, msg.DistanceFeet)
				lastLogTime = time.Now()
				lastCM = cm
			}
		}
	}
}

func round2(v float64) float64 { return math.Round(v*100) / 100 }
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
func errString(err error) string {
	if err == nil {
		return ""
	}
	// scrub generic errors
	switch {
	case errors.Is(err, os.ErrDeadlineExceeded):
		return "timeout"
	case errors.Is(err, os.ErrInvalid):
		return "invalid"
	default:
		return err.Error()
	}
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
