#!/bin/bash
# Hardware Optimization Script for Omega1 Robot
# Optimizes Raspberry Pi for low latency and reliability

set -e

echo "🔧 Omega1 Hardware Optimization Script"
echo "======================================"

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "❌ This script is designed for Raspberry Pi hardware"
    exit 1
fi

echo "✅ Running on Raspberry Pi hardware"

# Function to backup original config
backup_config() {
    local config_file="$1"
    if [ -f "$config_file" ]; then
        cp "$config_file" "${config_file}.backup.$(date +%Y%m%d_%H%M%S)"
        echo "📋 Backed up $config_file"
    fi
}

# Function to set config value
set_config() {
    local config_file="$1"
    local key="$2"
    local value="$3"
    
    if grep -q "^$key=" "$config_file" 2>/dev/null; then
        sed -i "s/^$key=.*/$key=$value/" "$config_file"
    else
        echo "$key=$value" >> "$config_file"
    fi
    echo "⚙️  Set $key=$value in $config_file"
}

echo ""
echo "🎯 Optimizing System Performance..."

# 1. CPU Governor Optimization
echo "📈 Setting CPU governor to performance mode..."
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null

# Make it persistent
backup_config /etc/default/cpufrequtils
cat > /etc/default/cpufrequtils << EOF
ENABLE=true
GOVERNOR=performance
MAX_SPEED=0
MIN_SPEED=0
EOF

# 2. GPU Memory Split Optimization
echo "🎮 Optimizing GPU memory split..."
backup_config /boot/config.txt

# Set GPU memory split for camera + minimal desktop
set_config /boot/config.txt "gpu_mem" "128"

# Enable camera
set_config /boot/config.txt "start_x" "1"
set_config /boot/config.txt "camera_auto_detect" "1"

# 3. Network Optimization
echo "🌐 Optimizing network settings..."
backup_config /etc/sysctl.conf

# TCP optimizations for low latency
cat >> /etc/sysctl.conf << EOF

# Omega1 Robot Network Optimizations
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.ipv4.tcp_rmem = 4096 65536 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216
net.core.netdev_max_backlog = 5000
net.ipv4.tcp_congestion_control = bbr
net.ipv4.tcp_fastopen = 3
net.ipv4.tcp_tw_reuse = 1
net.ipv4.tcp_fin_timeout = 15
net.ipv4.tcp_keepalive_time = 600
net.ipv4.tcp_keepalive_intvl = 30
net.ipv4.tcp_keepalive_probes = 3
EOF

# 4. GPIO Optimization
echo "🔌 Optimizing GPIO settings..."
backup_config /boot/config.txt

# GPIO optimizations
set_config /boot/config.txt "gpio" "gpio=18,23,24,25=op,dh"
set_config /boot/config.txt "dtparam" "spi=on"
set_config /boot/config.txt "dtparam" "i2c_arm=on"

# 5. Camera Optimization
echo "📹 Optimizing camera settings..."
backup_config /boot/config.txt

# Camera optimizations for low latency
set_config /boot/config.txt "camera_auto_detect" "1"
set_config /boot/config.txt "start_x" "1"

# Disable camera LED (optional)
set_config /boot/config.txt "disable_camera_led" "1"

# 6. USB Optimization
echo "🔌 Optimizing USB settings..."
backup_config /boot/cmdline.txt

# Add USB optimizations to cmdline
if ! grep -q "usbhid.mousepoll" /boot/cmdline.txt; then
    sed -i 's/$/ usbhid.mousepoll=0/' /boot/cmdline.txt
fi

# 7. Memory Optimization
echo "💾 Optimizing memory settings..."
backup_config /boot/config.txt

# Memory optimizations
set_config /boot/config.txt "arm_freq" "1500"
set_config /boot/config.txt "gpu_freq" "500"
set_config /boot/config.txt "core_freq" "500"
set_config /boot/config.txt "sdram_freq" "500"

# 8. Real-time Priority for Robot Services
echo "⚡ Setting up real-time priorities..."

# Create systemd service optimizations
cat > /etc/systemd/system/omega1-optimize.service << EOF
[Unit]
Description=Omega1 Robot Performance Optimization
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo -1 > /proc/sys/kernel/sched_rt_runtime_us'
ExecStart=/bin/bash -c 'echo 1000000 > /proc/sys/kernel/sched_rt_period_us'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl enable omega1-optimize.service

# 9. Create optimized camera configuration
echo "📸 Creating optimized camera configuration..."
mkdir -p /home/omega1/.config/omega1

cat > /home/omega1/.config/omega1/camera.conf << EOF
# Omega1 Camera Optimization Configuration
[Camera]
backend = picamera2
width = 640
height = 480
fps = 30
format = RGB888
warmup_timeout = 2.0
warmup_frames = 2

[Performance]
thread_priority = high
buffer_size = 2
drop_frames = true
EOF

# 10. GPIO Performance Optimization
echo "🔌 Creating GPIO performance configuration..."
cat > /home/omega1/.config/omega1/gpio.conf << EOF
# Omega1 GPIO Performance Configuration
[GPIO]
backend = lgpio
poll_interval = 10
debounce_ms = 5
interrupt_mode = true

[Sensors]
ultrasonic_interval = 50
line_tracker_interval = 20
motor_pwm_frequency = 1000
EOF

# 11. Network Interface Optimization
echo "🌐 Optimizing network interfaces..."

# Optimize WiFi
if command -v iwconfig >/dev/null 2>&1; then
    cat > /etc/network/interfaces.d/omega1-wifi << EOF
# Omega1 WiFi Optimization
auto wlan0
iface wlan0 inet dhcp
    wireless-power off
    wireless-mode managed
    wireless-channel auto
EOF
fi

# 12. Create performance monitoring script
echo "📊 Creating performance monitoring script..."
cat > /home/omega1/omega1-monitor.sh << 'EOF'
#!/bin/bash
# Omega1 Performance Monitor

echo "🤖 Omega1 Performance Status"
echo "============================="

echo "📈 CPU Status:"
echo "  Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "  Frequency: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) Hz"
echo "  Temperature: $(vcgencmd measure_temp)"

echo ""
echo "🎮 GPU Status:"
echo "  Memory Split: $(vcgencmd get_mem gpu)"
echo "  Frequency: $(vcgencmd measure_clock gpu) Hz"

echo ""
echo "📹 Camera Status:"
echo "  Camera Enabled: $(vcgencmd get_camera)"
echo "  Camera Detected: $(ls /dev/video* 2>/dev/null | wc -l) devices"

echo ""
echo "🌐 Network Status:"
echo "  WiFi Signal: $(iwconfig wlan0 2>/dev/null | grep 'Signal level' || echo 'Not available')"
echo "  Tailscale Status: $(tailscale status --json 2>/dev/null | jq -r '.Self.Online' || echo 'Not available')"

echo ""
echo "🔌 GPIO Status:"
echo "  GPIO Available: $(ls /dev/gpiochip* 2>/dev/null | wc -l) chips"

echo ""
echo "⚡ Real-time Status:"
echo "  RT Runtime: $(cat /proc/sys/kernel/sched_rt_runtime_us) μs"
echo "  RT Period: $(cat /proc/sys/kernel/sched_rt_period_us) μs"
EOF

chmod +x /home/omega1/omega1-monitor.sh

# 13. Create startup optimization script
echo "🚀 Creating startup optimization script..."
cat > /home/omega1/omega1-startup.sh << 'EOF'
#!/bin/bash
# Omega1 Startup Optimization

# Set CPU governor
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null

# Optimize network buffers
sudo sysctl -p /etc/sysctl.conf

# Set real-time priorities
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us > /dev/null
echo 1000000 | sudo tee /proc/sys/kernel/sched_rt_period_us > /dev/null

echo "✅ Omega1 hardware optimizations applied"
EOF

chmod +x /home/omega1/omega1-startup.sh

# 14. Create systemd service for startup optimization
cat > /etc/systemd/system/omega1-startup.service << EOF
[Unit]
Description=Omega1 Startup Optimization
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/home/omega1/omega1-startup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

systemctl enable omega1-startup.service

echo ""
echo "✅ Hardware optimization complete!"
echo ""
echo "📋 Summary of optimizations:"
echo "  • CPU governor set to performance mode"
echo "  • GPU memory optimized for camera"
echo "  • Network settings optimized for low latency"
echo "  • GPIO configured for high performance"
echo "  • Camera settings optimized"
echo "  • Real-time priorities enabled"
echo "  • Performance monitoring script created"
echo ""
echo "🔄 Reboot required for all changes to take effect:"
echo "   sudo reboot"
echo ""
echo "📊 Monitor performance with:"
echo "   ./omega1-monitor.sh"
echo ""
echo "🎯 Expected improvements:"
echo "  • Lower WebSocket latency"
echo "  • Faster sensor response"
echo "  • Smoother video streaming"
echo "  • More reliable GPIO operations"
echo "  • Better mobile hotspot performance"

