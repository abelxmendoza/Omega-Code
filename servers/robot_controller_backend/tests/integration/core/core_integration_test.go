// File: /Omega-Code/servers/robot-controller-backend/tests/integration/core/core_integration_test.go

package core

import (
    "net/http"
    "net/http/httptest"
    "testing"
    "github.com/stretchr/testify/assert"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/commands"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/common"
)

func TestStartServer(t *testing.T) {
    os.Setenv("CERT_PATH", "/path/to/cert.pem")
    os.Setenv("KEY_PATH", "/path/to/key.pem")

    go StartServer()
    
    req, err := http.NewRequest("GET", "/command", nil)
    assert.NoError(t, err)

    rr := httptest.NewRecorder()
    handler := http.HandlerFunc(commands.HandleCommand)

    handler.ServeHTTP(rr, req)

    assert.Equal(t, http.StatusOK, rr.Code)
}

