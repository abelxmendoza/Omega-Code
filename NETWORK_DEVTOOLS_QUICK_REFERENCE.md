# Network DevTools Quick Reference

## 🚀 Quick Start

1. Open browser: `http://localhost:3000/network`
2. Press `F12` → "Network" tab
3. Watch for `GET /api/network` calls every 5 seconds

## 📡 Main API Call

**Endpoint:** `GET /api/network`

**Frequency:** Every 5 seconds (auto-refresh)

**What to Look For:**
- ✅ Status: `200 OK`
- ✅ Duration: < 200ms
- ✅ Response includes: `mode`, `ip`, `services`, `vpn_status`

## 🎯 Key Requests

| Action | Method | Endpoint | When |
|--------|--------|----------|------|
| Get Status | GET | `/api/network` | Every 5s |
| Switch Mode | POST | `/api/network/mode` | Button click |
| Validate | POST | `/api/network/validate` | Button click |
| Scan WiFi | GET | `/api/network/wifi/scan` | If implemented |

## 🔍 Filter Tips

- Type `network` in filter → See only network API calls
- Type `200` → See successful requests
- Type `503` → See offline/error states

## 📊 Expected Response

```json
{
  "ok": true,
  "mode": "ap",
  "ip": "192.168.4.1",
  "services_running": {
    "hostapd": {"status": "active"},
    "dnsmasq": {"status": "active"}
  }
}
```

## ⚠️ Common Issues

- **503 Status:** Robot backend offline
- **Timeout:** Network connectivity issue
- **500 Status:** Backend error (check logs)
