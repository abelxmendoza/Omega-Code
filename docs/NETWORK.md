# Network Features Guide

## Overview

Navigate to `http://localhost:3000/network` (or your deployed URL) to access the network management page.

## Network API Endpoints

| Action | Method | Endpoint | Frequency |
|--------|--------|----------|-----------|
| Get status | GET | `/api/network` | Auto every 5s |
| Switch mode | POST | `/api/network/mode` | On button click |
| Validate config | POST | `/api/network/validate` | On button click |
| Scan Wi-Fi | GET | `/api/network/wifi/scan` | On request |
| Net summary | GET | `/api/net/summary` | Auto every 8s |

## Network Status Response

```json
{
  "ok": true,
  "mode": "ap",
  "interface": "wlan0",
  "ssid": "Omega1-AP",
  "ip": "192.168.4.1",
  "rssi": -45,
  "vpn_status": {
    "enabled": true,
    "ip": "100.93.225.61",
    "status": "connected",
    "hostname": "omega1-1.hartley-ghost.ts.net"
  },
  "services_running": {
    "hostapd": { "status": "active", "enabled": true },
    "dnsmasq": { "status": "active", "enabled": true },
    "wpa_supplicant": { "status": "inactive", "enabled": false }
  },
  "errors": [],
  "warnings": []
}
```

When robot is offline:
```json
{ "offline": true, "message": "Robot backend unavailable: /api/network", "status": 503 }
```

## Mode Switching

```bash
# Switch to AP mode
POST /api/network/mode
{ "mode": "ap" }

# Switch to client mode
POST /api/network/mode
{ "mode": "client" }
```

Response:
```json
{ "ok": true, "mode": "ap", "message": "Network mode set to ap" }
```

Mode switch takes ~2-5 seconds.

## Validation

```bash
POST /api/network/validate
```

Response:
```json
{
  "ok": true,
  "valid": true,
  "results": {
    "ap_config_exists": true,
    "hostapd_running": true,
    "wlan0_static_ip": true,
    "dhcp_service_running": true
  }
}
```

---

## Debugging in Browser DevTools

Open DevTools: `F12` → Network tab (Chrome/Firefox/Edge) or `Cmd+Option+I` (Mac).

### What healthy traffic looks like

```
Time    Method  URL                     Status  Time
─────────────────────────────────────────────────────
0ms     GET     /api/network           200     45ms
5.1s    GET     /api/network           200     52ms
15.3s   POST    /api/network/mode      200     2.1s
22.6s   POST    /api/network/validate  200     120ms
```

### Filters

- Type `network` → see only network API calls
- Type `503` → find offline/error states
- Check "Preserve log" to keep history across navigations

### Console logs (normal operation)

```javascript
[NetworkWizard] Fetching network status...
[NetworkWizard] Network status received: { mode: "ap", ... }
[NetworkWizard] Switching to AP mode...
[NetworkWizard] Mode switch successful
```

### Console logs (errors)

```javascript
[Robot OFFLINE] Blocked fetch: /api/network
[NetworkWizard] Failed to fetch network status: NetworkError
```

---

## Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| All requests 503 | Robot backend offline | Start backend, check network |
| Requests timing out | Poor connectivity | Check `rssi` in response, check firewall |
| Mode switch fails 500 | Permission denied | Check backend logs, verify Pi permissions |
| Missing fields in response | Backend error | Check Pi logs |
