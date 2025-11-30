# Lighting Server Troubleshooting Guide

## Common Issues and Solutions

### 1. Guest Wi-Fi or Firewall Restrictions

**Problem**: Connection works on home Wi-Fi but not on guest Wi-Fi.

**Solution**: Ensure firewall allows WebSocket traffic on port 8082.

```bash
# Check firewall status
sudo ufw status

# Allow WebSocket port (if using ufw)
sudo ufw allow 8082/tcp

# For iptables (if not using ufw)
sudo iptables -A INPUT -p tcp --dport 8082 -j ACCEPT
sudo iptables-save
```

**Verify**: Test connectivity from another machine:
```bash
curl http://100.93.225.61:8082/health  # Tailscale IP
curl http://192.168.6.164:8082/health  # Local IP
```

### 2. Tailscale Configuration

**Problem**: Tailscale not routing traffic correctly.

**Solution**: Verify Tailscale status and restart if needed.

```bash
# Check Tailscale status
tailscale status

# Verify both devices are connected
tailscale ping omega1-1.hartley-ghost.ts.net

# Restart Tailscale if needed
sudo tailscale down
sudo tailscale up

# Check Tailscale IP
tailscale ip
```

### 3. Ping vs. Command Messages

**Problem**: Only ping messages arrive, lighting commands don't.

**Symptoms**:
- Server logs show only `{"type":"ping"}` messages
- No `📨 [CMD] Received lighting command` logs
- Frontend shows "connected" but commands don't work

**Debug Steps**:

1. **Check frontend console**:
   - Open browser DevTools → Console
   - Look for `[LedModal] 📤 Sending message:` logs
   - Verify WebSocket state: `ws.current?.readyState === WebSocket.OPEN`

2. **Check server logs**:
   ```bash
   # Look for command messages
   grep "CMD.*lighting command" /path/to/server.log
   
   # Check for JSON parse errors
   grep "JSON unmarshal failed" /path/to/server.log
   ```

3. **Verify message format**:
   - Frontend should send: `{"color":"#ffffff","mode":"single","pattern":"static","interval":0,"brightness":0.35}`
   - Server expects: `LightingCommand` struct with these fields

4. **Test manually**:
   ```bash
   # Connect via websocat (install: sudo apt install websocat)
   websocat ws://100.93.225.61:8082/lighting
   
   # Then send:
   {"color":"#ffffff","mode":"single","pattern":"static","interval":0,"brightness":0.35}
   ```

### 4. Script Permissions

**Problem**: LED control script not executable.

**Solution**: Ensure script has execute permissions.

```bash
# Check permissions
ls -l /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting/run_led.sh

# Make executable
sudo chmod +x /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting/run_led.sh

# Verify
./run_led.sh --help  # Should not show "Permission denied"
```

### 5. Hardware Configuration

**Problem**: LEDs not physically connected or GPIO not accessible.

**Solution**: Verify hardware setup.

```bash
# Check if running on Raspberry Pi
cat /proc/cpuinfo | grep Model

# Test GPIO access (if using GPIO pins)
sudo python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO OK')"

# Check if rpi-ws281x is installed
python3 -c "import rpi_ws281x; print('rpi-ws281x OK')"

# Test LED script directly
cd /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting
sudo ./run_led.sh ffffff 000000 alternate single static 0 0.35
```

### 6. Python Dependencies

**Problem**: Missing Python modules (e.g., `rpi-ws281x`).

**Solution**: Install dependencies.

```bash
# Use the installer script
cd /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting
./install_led_deps.sh

# Or manually
cd /home/omega1/Omega-Code/servers/robot_controller_backend
source venv/bin/activate
pip install rpi-ws281x
```

### 7. Network Connectivity Issues

**Problem**: Connection drops frequently, especially on mobile hotspot.

**Solution**: Check network stability and use Tailscale.

```bash
# Test latency
ping -c 10 100.93.225.61  # Tailscale IP

# Test WebSocket connectivity
curl -v --http1.1 -H "Connection: Upgrade" -H "Upgrade: websocket" \
  http://100.93.225.61:8082/lighting

# Check for packet loss
mtr --report --report-cycles 10 100.93.225.61
```

### 8. Server Not Running

**Problem**: Server not listening on port 8082.

**Solution**: Start the server and verify it's running.

```bash
# Check if port is in use
sudo netstat -tlnp | grep 8082
# or
sudo ss -tlnp | grep 8082

# Start the server
cd /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting
go run main_lighting.go

# Check server logs for startup messages
# Should see: "🎯 [READY] Server is now listening and accepting connections!"
```

## Diagnostic Script

Run the diagnostic script to check all common issues:

```bash
cd /home/omega1/Omega-Code/servers/robot_controller_backend/controllers/lighting
./diagnose.sh
```

## Quick Checklist

- [ ] Firewall allows port 8082
- [ ] Tailscale is running and connected
- [ ] Server is running and listening on port 8082
- [ ] Script has execute permissions (`chmod +x`)
- [ ] Python dependencies installed (`rpi-ws281x`)
- [ ] Hardware (LEDs) physically connected
- [ ] Frontend WebSocket shows "connected" status
- [ ] Browser console shows command messages being sent
- [ ] Server logs show command messages being received

## Getting Help

If issues persist:

1. Collect logs:
   ```bash
   # Server logs
   journalctl -u lighting-server -n 100 > lighting-server.log
   
   # Browser console logs (copy from DevTools)
   ```

2. Run diagnostics:
   ```bash
   ./diagnose.sh > diagnostics.txt
   ```

3. Check network:
   ```bash
   tailscale status > tailscale-status.txt
   netstat -tlnp > network-ports.txt
   ```

