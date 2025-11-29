# Quick Start: Frontend-Backend-Hardware Integration

## 🚀 Quick Commands

### SSH into omega1

```bash
# Via Tailscale (recommended)
./scripts/ssh_omega1.sh tailscale

# Via LAN
./scripts/ssh_omega1.sh lan
```

### Verify Integration

```bash
# Test everything from MacBook
./scripts/verify_integration.sh tailscale
```

### Test Hardware on Pi

```bash
# SSH into omega1 first
./scripts/ssh_omega1.sh tailscale

# Then run hardware test
~/Omega-Code/scripts/test_hardware_on_pi.sh
```

## 📋 Checklist

### On MacBook (Development)

- [ ] Frontend environment configured (`ui/robot-controller-ui/.env.local`)
- [ ] Can SSH into omega1 via Tailscale
- [ ] Can SSH into omega1 via LAN
- [ ] Integration verification passes

### On omega1 (Raspberry Pi)

- [ ] Backend environment configured (`servers/robot_controller_backend/.env`)
- [ ] Gateway API running (port 7070)
- [ ] Movement WebSocket running (port 8081)
- [ ] Video server running (port 5000)
- [ ] Hardware test passes
- [ ] GPIO permissions configured
- [ ] Camera enabled

## 🔧 Common Issues & Fixes

### Can't SSH into omega1

**Problem:** SSH connection fails

**Solutions:**
1. Check Tailscale status: `tailscale status`
2. Verify hostname: `ping omega1-tailscale`
3. Check SSH service: `ssh omega1-tailscale "systemctl status ssh"`

### Frontend can't connect to backend

**Problem:** WebSocket connection fails

**Solutions:**
1. Verify gateway is running: `curl http://100.93.225.61:7070/health`
2. Check firewall: `sudo ufw allow 7070/tcp`
3. Verify environment variables match

### Hardware not responding

**Problem:** Motors/sensors don't work

**Solutions:**
1. Check GPIO permissions: `groups | grep gpio`
2. Add user to gpio group: `sudo usermod -a -G gpio $USER`
3. Verify simulation mode is OFF: `ROBOT_SIM=0` in `.env`

## 📚 Next Steps

1. Read [INTEGRATION_VERIFICATION.md](INTEGRATION_VERIFICATION.md) for detailed guide
2. Check [ENVIRONMENT_VARIABLES.md](ENVIRONMENT_VARIABLES.md) for configuration
3. Review [README.md](README.md) for architecture overview

## 🆘 Need Help?

Run the verification script to see what's failing:

```bash
./scripts/verify_integration.sh tailscale
```

This will test:
- ✅ Network connectivity
- ✅ Gateway API endpoints
- ✅ WebSocket connections
- ✅ Service ports
- ✅ ROS2 integration
- ✅ Hardware status

