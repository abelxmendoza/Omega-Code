// File: /Omega-Code/servers/robot_controller_backend/tests/unit/core/core_test.go

package core

import (
    "bytes"
    "net/http"
    "net/http/httptest"
    "testing"
    "github.com/gorilla/websocket"
    "github.com/stretchr/testify/assert"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/commands"
)

// Mock for websocket connection
func mockWebSocketConn() *websocket.Conn {
    s := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
        upgrader := websocket.Upgrader{
            ReadBufferSize:  1024,
            WriteBufferSize: 1024,
            CheckOrigin:     func(r *http.Request) bool { return true },
        }
        c, err := upgrader.Upgrade(w, r, nil)
        if err != nil {
            t.Fatal("Failed to upgrade WebSocket:", err)
        }
        defer c.Close()
    }))
    defer s.Close()

    u := "ws" + s.URL[len("http"):]
    c, _, err := websocket.DefaultDialer.Dial(u, nil)
    if err != nil {
        t.Fatal("Failed to dial WebSocket:", err)
    }
    return c
}

func TestHandleConnections(t *testing.T) {
    req, err := http.NewRequest("GET", "/ws", nil)
    assert.NoError(t, err)

    rr := httptest.NewRecorder()
    handler := http.HandlerFunc(HandleConnections)

    handler.ServeHTTP(rr, req)

    assert.Equal(t, http.StatusSwitchingProtocols, rr.Code)
}

func TestHandleWebSocketUltrasonicSensor(t *testing.T) {
    ws := mockWebSocketConn()
    go HandleWebSocketUltrasonicSensor(ws)

    var data commands.UltrasonicData
    err := ws.ReadJSON(&data)
    assert.NoError(t, err)
    assert.Greater(t, data.Distance, 0)
}

func TestHandleWebSocketLineTracking(t *testing.T) {
    ws := mockWebSocketConn()
    go HandleWebSocketLineTracking(ws)

    var data commands.LineTrackingData
    err := ws.ReadJSON(&data)
    assert.NoError(t, err)
    assert.True(t, data.IR01 == 0 || data.IR01 == 1)
    assert.True(t, data.IR02 == 0 || data.IR02 == 1)
    assert.True(t, data.IR03 == 0 || data.IR03 == 1)
}

func TestHandleWebSocketLEDControl(t *testing.T) {
    ws := mockWebSocketConn()

    msg := map[string]interface{}{
        "color":    float64(0xFF0000),
        "mode":     "single",
        "pattern":  "static",
        "interval": float64(1000),
    }

    err := ws.WriteJSON(msg)
    assert.NoError(t, err)

    go HandleWebSocketLEDControl(ws, msg)

    var data map[string]interface{}
    err = ws.ReadJSON(&data)
    assert.NoError(t, err)
    assert.Equal(t, float64(0xFF0000), data["color"])
    assert.Equal(t, "single", data["mode"])
    assert.Equal(t, "static", data["pattern"])
    assert.Equal(t, float64(1000), data["interval"])
}


