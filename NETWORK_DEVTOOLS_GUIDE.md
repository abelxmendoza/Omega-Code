# Network Features in Browser DevTools Guide

## 🔍 How to View Network Features in Browser DevTools

### Step 1: Open the Network Page

Navigate to: **`http://localhost:3000/network`** (or your deployed URL)

### Step 2: Open Browser DevTools

**Chrome/Edge:**
- Press `F12` or `Cmd+Option+I` (Mac) / `Ctrl+Shift+I` (Windows/Linux)
- Or right-click → "Inspect" → "Network" tab

**Firefox:**
- Press `F12` or `Cmd+Option+I` (Mac) / `Ctrl+Shift+I` (Windows/Linux)
- Or right-click → "Inspect" → "Network" tab

**Safari:**
- Enable Developer menu: Preferences → Advanced → "Show Develop menu"
- Press `Cmd+Option+I` → "Network" tab

---

## 📡 Network API Calls You'll See

### 1. **Unified Network Summary** (Main Call)

**Request:**
```
GET /api/network
```

**Appears in DevTools as:**
- **Name:** `network`
- **Method:** `GET`
- **Status:** `200 OK` (or `503` if robot offline)
- **Type:** `fetch` or `xhr`
- **Initiator:** `OmegaNetworkWizard.tsx` (line ~60)

**Request Headers:**
```
GET /api/network HTTP/1.1
Host: omega1.local:8080
Accept: application/json
```

**Response (Success):**
```json
{
  "ok": true,
  "mode": "ap",
  "interface": "wlan0",
  "ssid": "Omega1-AP",
  "ip": "192.168.4.1",
  "gateway": null,
  "rssi": -45,
  "vpn_status": {
    "enabled": true,
    "ip": "100.93.225.61",
    "status": "connected",
    "hostname": "omega1-1.hartley-ghost.ts.net"
  },
  "pan_status": {
    "enabled": false,
    "status": "disconnected"
  },
  "services_running": {
    "hostapd": {
      "status": "active",
      "enabled": true
    },
    "dnsmasq": {
      "status": "active",
      "enabled": true
    },
    "dhcpcd": {
      "status": "active",
      "enabled": true
    },
    "wpa_supplicant": {
      "status": "inactive",
      "enabled": false
    }
  },
  "ap_config": {
    "ssid": "Omega1-AP",
    "ip": "192.168.4.1"
  },
  "errors": [],
  "warnings": [],
  "last_updated": "2025-01-XX..."
}
```

**Response (Robot Offline):**
```json
{
  "offline": true,
  "message": "Robot backend unavailable: /api/network",
  "status": 503
}
```

**Timing:**
- **Duration:** ~50-200ms (local network)
- **Repeats:** Every 5 seconds (auto-refresh)

---

### 2. **Mode Switching** (When Clicking AP/Client Buttons)

**Request:**
```
POST /api/network/mode
```

**Appears in DevTools as:**
- **Name:** `mode`
- **Method:** `POST`
- **Status:** `200 OK` (or `500` on error)
- **Type:** `fetch`
- **Initiator:** `OmegaNetworkWizard.tsx` (line ~109)

**Request Payload:**
```json
{
  "mode": "ap"
}
```

**Request Headers:**
```
POST /api/network/mode HTTP/1.1
Host: omega1.local:8080
Content-Type: application/json
```

**Response (Success):**
```json
{
  "ok": true,
  "mode": "ap",
  "message": "Network mode set to ap"
}
```

**Response (Error):**
```json
{
  "detail": "Failed to switch network mode: Permission denied"
}
```

**Timing:**
- **Duration:** ~2-5 seconds (mode switch takes time)
- **One-time:** Only when button clicked

---

### 3. **Configuration Validation** (When Clicking Validate Button)

**Request:**
```
POST /api/network/validate
```

**Appears in DevTools as:**
- **Name:** `validate`
- **Method:** `POST`
- **Status:** `200 OK`
- **Type:** `fetch`

**Response:**
```json
{
  "ok": true,
  "valid": true,
  "results": {
    "ap_config_exists": true,
    "hostapd_running": true,
    "wlan0_static_ip": true,
    "dhcp_service_running": true
  },
  "message": "Network configuration is valid"
}
```

---

### 4. **Wi-Fi Network Scan** (If Implemented)

**Request:**
```
GET /api/network/wifi/scan
```

**Response:**
```json
{
  "ok": true,
  "networks": [
    {
      "ssid": "MyWiFi",
      "security": "WPA2",
      "signal": 85,
      "frequency": "2.4GHz"
    }
  ],
  "count": 1
}
```

---

## 🎨 Visual Layout in Browser

### Network Page Structure:

```
┌─────────────────────────────────────────────────┐
│ Network Management                              │
│ Manage network profiles and optimize            │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│ 🔌 Omega Network Wizard                         │
│                                                 │
│ Current Status:                                 │
│   Mode: AP MODE                                 │
│   IP Address: 192.168.4.1                      │
│   SSID: Omega1-AP                               │
│                                                 │
│ Services:                                       │
│   ✓ hostapd: active                            │
│   ✓ dnsmasq: active                            │
│   ✓ dhcpcd: active                             │
│   ✗ wpa_supplicant: inactive                   │
│                                                 │
│ [Enable AP Mode] [Enable Client Mode]          │
│ [Validate Configuration]                        │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│ Network Profiles                                │
│ [Profile selector with Tailscale/WiFi/Mobile] │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│ Mobile Connection Test                          │
│ [Quick Test] [Full Test]                        │
└─────────────────────────────────────────────────┘
```

---

## 🔍 Console Logs You'll See

### Normal Operation:

```javascript
// Initial load
[NetworkWizard] Fetching network status...
[NetworkWizard] Network status received: { mode: "ap", ip: "192.168.4.1", ... }

// Auto-refresh (every 5 seconds)
[NetworkWizard] Refreshing network status...

// Mode switch
[NetworkWizard] Switching to AP mode...
[NetworkWizard] Mode switch successful

// Validation
[NetworkWizard] Running validation...
[NetworkWizard] Validation results: { ap_config_exists: true, ... }
```

### Error Cases:

```javascript
// Robot offline
[Robot OFFLINE] Blocked fetch: /api/network
[NetworkWizard] Robot backend unavailable

// Network error
[NetworkWizard] Failed to fetch network status: NetworkError
[NetworkWizard] Error: Failed to switch network mode: Timeout
```

---

## 📊 Network Tab Filtering

### Filter by Type:
- **Fetch/XHR:** Shows all API calls
- **WS (WebSocket):** Shows WebSocket connections (if any)
- **Doc:** Shows page HTML

### Filter by Name:
- Type `network` to see only `/api/network` calls
- Type `mode` to see mode switching calls
- Type `validate` to see validation calls

### Filter by Status:
- **200:** Successful requests
- **503:** Robot offline
- **500:** Server errors
- **Timeout:** Network issues

---

## 🎯 Key Things to Look For

### ✅ Healthy Network:

1. **Regular 5-second intervals:**
   - `GET /api/network` every 5 seconds
   - Status: `200 OK`
   - Duration: < 200ms

2. **Complete response:**
   - All fields present (`mode`, `ip`, `services`, etc.)
   - No errors in `errors` array
   - Services show correct status

3. **Fast mode switching:**
   - `POST /api/network/mode` completes in < 5 seconds
   - Status: `200 OK`
   - Response includes success message

### ⚠️ Issues to Watch For:

1. **Robot Offline:**
   - Status: `503 Service Unavailable`
   - Response: `{ "offline": true }`
   - Console: `[Robot OFFLINE] Blocked fetch`

2. **Slow Responses:**
   - Duration: > 1 second
   - May indicate network issues
   - Check `rssi` in response

3. **Failed Mode Switches:**
   - Status: `500 Internal Server Error`
   - Response: `{ "detail": "Permission denied" }`
   - Check backend logs

4. **Missing Data:**
   - Response missing fields
   - `errors` array contains items
   - Services show `unknown` status

---

## 🧪 Testing in DevTools

### Test Mode Switching:

1. Open Network tab
2. Click "Enable AP Mode" button
3. Watch for `POST /api/network/mode` request
4. Check response for success/error
5. Verify status updates after 2 seconds

### Test Validation:

1. Click "Validate Configuration" button
2. Watch for `POST /api/network/validate` request
3. Check response `results` object
4. Verify all checks pass (`true`)

### Test Auto-Refresh:

1. Clear Network tab (right-click → "Clear")
2. Wait 5 seconds
3. Verify `GET /api/network` appears automatically
4. Check timing: should be ~5 seconds apart

---

## 📱 Mobile View

On mobile devices, the Network tab shows the same requests but:
- Smaller screen = scroll to see all
- Touch-friendly buttons
- Same API calls, same responses
- May show slower network times

---

## 🔗 Related Network Calls

The network page also makes these calls (visible in DevTools):

1. **Network Profile Summary:**
   - `GET /api/net/summary` (every 8 seconds)
   - Shows connection quality

2. **Mobile Connection Test:**
   - `GET /api/network/wifi/scan` (when testing)
   - Shows available Wi-Fi networks

---

## 💡 Pro Tips

1. **Use Preserve Log:**
   - Check "Preserve log" in Network tab
   - Keeps requests after page navigation

2. **Filter by Domain:**
   - Type `omega1.local` to see only robot API calls
   - Excludes Next.js internal calls

3. **Export HAR:**
   - Right-click → "Save all as HAR"
   - Share with team for debugging

4. **Throttle Network:**
   - Use "Slow 3G" throttling
   - Test how UI handles slow connections

5. **Monitor WebSocket:**
   - If using WebSocket bridge
   - Check WS tab for real-time updates

---

## 🎬 Example DevTools Session

```
Time    Method  URL                     Status  Type    Size    Time
─────────────────────────────────────────────────────────────────────
0ms     GET     /api/network           200     fetch   1.2KB   45ms
5.1s    GET     /api/network           200     fetch   1.2KB   52ms
10.2s   GET     /api/network           200     fetch   1.2KB   48ms
15.3s   POST    /api/network/mode      200     fetch   0.3KB   2.1s
17.5s   GET     /api/network           200     fetch   1.2KB   51ms
22.6s   POST    /api/network/validate  200     fetch   0.5KB   120ms
27.7s   GET     /api/network           200     fetch   1.2KB   49ms
```

This shows:
- ✅ Regular 5-second polling
- ✅ Mode switch took 2.1 seconds
- ✅ Validation took 120ms
- ✅ All requests successful (200)

---

## 🐛 Debugging Common Issues

### Issue: No network requests appearing

**Check:**
- Is the page loaded? (`/network`)
- Is DevTools Network tab open?
- Is "Preserve log" enabled?
- Is robot backend running?

### Issue: All requests showing 503

**Check:**
- Robot backend is offline
- `ROBOT_ENABLED=false` in environment
- Network connectivity to robot

### Issue: Requests timing out

**Check:**
- Network connection to robot
- Robot backend is running
- Firewall blocking requests
- Check `rssi` in response (signal strength)

---

## 📚 Summary

In Browser DevTools Network tab, you'll see:

1. **Main Call:** `GET /api/network` every 5 seconds
2. **Mode Switch:** `POST /api/network/mode` when clicking buttons
3. **Validation:** `POST /api/network/validate` when validating
4. **All responses:** JSON with comprehensive network status

The unified endpoint (`/api/network`) provides everything in one call, making it easy to monitor and debug network status!

