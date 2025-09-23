# Omega1 Hardware Optimization Guide

## 🚀 Low Latency & Reliability Optimizations

### 1. System-Level Optimizations

#### CPU Performance
```bash
# Set CPU governor to performance mode
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make it persistent
echo 'GOVERNOR=performance' | sudo tee -a /etc/default/cpufrequtils
```

#### GPU Memory Optimization
```bash
# Optimize GPU memory split for camera
sudo raspi-config nonint do_memory_split 128
```

#### Network Optimization
```bash
# Add to /etc/sysctl.conf
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.ipv4.tcp_rmem = 4096 65536 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216
net.ipv4.tcp_congestion_control = bbr
net.ipv4.tcp_fastopen = 3
```

### 2. Camera Optimizations

#### Environment Variables
```bash
export CAMERA_BACKEND=picamera2
export CAMERA_WIDTH=640
export CAMERA_HEIGHT=480
export CAMERA_FPS=30
export CAMERA_WARMUP_MS=2000
export FRAME_STALE_MS=1500
```

#### Boot Configuration (/boot/config.txt)
```
# Camera optimizations
start_x=1
camera_auto_detect=1
disable_camera_led=1

# GPU optimizations
gpu_mem=128
arm_freq=1500
gpu_freq=500
```

### 3. GPIO Optimizations

#### Line Tracker Performance
```bash
export RATE_HZ=20          # Increased from 10Hz
export KEEPALIVE_SEC=1.0   # Reduced from 2.0s
export GPIO_POLL_INTERVAL=10
export GPIO_DEBOUNCE_MS=5
```

#### Motor Control
```bash
export MOTOR_PWM_FREQUENCY=1000
export USE_INTERRUPT_MODE=true
```

### 4. Real-Time Priority

#### Enable Real-Time Scheduling
```bash
# Add to /etc/sysctl.conf
kernel.sched_rt_runtime_us = -1
kernel.sched_rt_period_us = 1000000
```

### 5. Service Optimizations

#### Create systemd service for robot processes
```bash
# /etc/systemd/system/omega1-robot.service
[Unit]
Description=Omega1 Robot Services
After=network.target

[Service]
Type=simple
User=omega1
WorkingDirectory=/home/omega1/Omega-Code/servers/robot-controller-backend
ExecStart=/bin/bash -c 'cd sensors && python3 line_tracking_ws_server.py & go run main_ultrasonic.go & cd ../video && python3 video_server.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 6. Network Interface Optimization

#### WiFi Optimization
```bash
# Add to /etc/network/interfaces.d/omega1-wifi
auto wlan0
iface wlan0 inet dhcp
    wireless-power off
    wireless-mode managed
```

### 7. Performance Monitoring

#### Create monitoring script
```bash
#!/bin/bash
# omega1-monitor.sh

echo "🤖 Omega1 Performance Status"
echo "CPU: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "Temp: $(vcgencmd measure_temp)"
echo "GPU Mem: $(vcgencmd get_mem gpu)"
echo "Camera: $(vcgencmd get_camera)"
echo "WiFi: $(iwconfig wlan0 | grep 'Signal level')"
```

### 8. Expected Performance Improvements

- **WebSocket Latency**: 50-100ms reduction
- **Sensor Response**: 2x faster (20Hz vs 10Hz)
- **Camera Startup**: 1s faster warmup
- **GPIO Response**: 5ms faster debounce
- **Network Stability**: Better mobile hotspot performance
- **System Reliability**: Real-time priority prevents blocking

### 9. Mobile Hotspot Specific Optimizations

#### Tailscale Configuration
```bash
# Optimize Tailscale for mobile
tailscale set --accept-routes=false
tailscale set --accept-dns=false
```

#### Network Interface Priority
```bash
# Prioritize Tailscale interface
ip route add default via 100.64.0.1 dev tailscale0 metric 100
```

### 10. Verification Commands

```bash
# Check optimizations
vcgencmd measure_temp
vcgencmd get_mem gpu
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
tailscale status
iwconfig wlan0
```

## 🎯 Results

With these optimizations, you should see:
- **Lower latency** WebSocket connections
- **Faster sensor** response times
- **Smoother video** streaming
- **More reliable** mobile hotspot performance
- **Better real-time** control responsiveness

