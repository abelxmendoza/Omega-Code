# 🚀 Hardware Performance Optimization Guide

## Overview

This guide covers comprehensive hardware performance optimizations for the Omega-Code robot controller system, focusing on maximizing performance while maintaining reliability and power efficiency.

## 🎯 Performance Targets

| Component | Target Performance | Optimization Level |
|-----------|-------------------|-------------------|
| **GPIO Operations** | <1ms response time | **Critical** |
| **Motor Control** | <10ms response time | **High** |
| **Camera Capture** | 30+ FPS stable | **High** |
| **Sensor Reading** | <5ms response time | **Medium** |
| **System Resources** | <80% CPU usage | **Medium** |

## 🔧 System-Level Optimizations

### 1. CPU Performance Tuning

#### Set CPU Governor to Performance Mode
```bash
# Set all CPU cores to performance mode
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make it persistent across reboots
echo 'GOVERNOR=performance' | sudo tee -a /etc/default/cpufrequtils
```

#### CPU Frequency Scaling
```bash
# Set minimum CPU frequency
echo 1500000 | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq

# Disable CPU idle states for critical cores
echo 1 | sudo tee /sys/devices/system/cpu/cpu0/cpuidle/state*/disable
```

### 2. Memory Optimization

#### GPU Memory Split
```bash
# Optimize GPU memory for camera operations
sudo raspi-config nonint do_memory_split 128

# Add to /boot/config.txt
gpu_mem=128
```

#### Memory Management
```bash
# Increase shared memory for camera buffers
echo "tmpfs /tmp tmpfs defaults,size=100M 0 0" | sudo tee -a /etc/fstab

# Optimize memory allocation
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
echo 'vm.vfs_cache_pressure=50' | sudo tee -a /etc/sysctl.conf
```

### 3. Network Optimization

#### TCP/UDP Tuning
```bash
# Add to /etc/sysctl.conf
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.ipv4.tcp_rmem = 4096 65536 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216
net.ipv4.tcp_congestion_control = bbr
net.ipv4.tcp_fastopen = 3
net.core.netdev_max_backlog = 5000
```

#### WiFi Optimization
```bash
# Optimize WiFi for low latency
echo 'options 8192cu rtw_power_mgnt=0 rtw_enusbss=0' | sudo tee -a /etc/modprobe.d/8192cu.conf
```

## 🎮 GPIO Optimizations

### 1. High-Performance GPIO Configuration

#### Use Hardware PWM
```python
from hardware.optimization.gpio_optimizer import gpio_optimizer

# Configure motor PWM pins for hardware PWM
gpio_optimizer.configure_pin(18, 'pwm', frequency=1000)  # Motor 1 PWM
gpio_optimizer.configure_pin(19, 'pwm', frequency=1000)  # Motor 2 PWM

# Configure servo PWM pins
gpio_optimizer.configure_pin(12, 'pwm', frequency=50)   # Servo 1 PWM
gpio_optimizer.configure_pin(13, 'pwm', frequency=50)   # Servo 2 PWM
```

#### Interrupt-Driven Sensor Reading
```python
# Configure ultrasonic sensor for interrupt-driven reading
gpio_optimizer.configure_pin(20, 'input', pull='up', edge='both')
gpio_optimizer.configure_pin(21, 'output')

# Set interrupt handler
def ultrasonic_interrupt_handler(level, timestamp):
    # Process ultrasonic reading
    pass

gpio_optimizer.set_interrupt_handler(20, ultrasonic_interrupt_handler)
```

### 2. GPIO Performance Settings

#### Environment Variables
```bash
# GPIO optimization settings
export GPIO_POLL_INTERVAL=1        # 1ms polling interval
export GPIO_DEBOUNCE_MS=5         # 5ms debounce time
export GPIO_CACHE_TIMEOUT=10      # 10ms cache timeout
export GPIO_INTERRUPT_MODE=true   # Enable interrupt mode
```

## 🚗 Motor Control Optimizations

### 1. Motor Configuration

#### High-Performance Motor Setup
```python
from hardware.optimization.motor_optimizer import motor_optimizer, MotorConfig, MotorType

# Configure motors for optimal performance
motor_configs = [
    MotorConfig(
        motor_id="front_left",
        motor_type=MotorType.DC_BRUSHED,
        pwm_pin=18,
        direction_pin=23,
        enable_pin=24,
        max_speed=4095,
        acceleration_rate=200.0,  # Fast acceleration
        deceleration_rate=300.0, # Fast deceleration
        deadband=50
    ),
    MotorConfig(
        motor_id="front_right",
        motor_type=MotorType.DC_BRUSHED,
        pwm_pin=19,
        direction_pin=25,
        enable_pin=8,
        max_speed=4095,
        acceleration_rate=200.0,
        deceleration_rate=300.0,
        deadband=50
    )
]

# Add motors to optimizer
for config in motor_configs:
    motor_optimizer.add_motor(config)
```

#### Motor Synchronization
```python
# Synchronize all motors for smooth movement
motor_ids = ["front_left", "front_right", "rear_left", "rear_right"]
motor_optimizer.sync_motors(motor_ids, target_speed=2000)
```

### 2. Motor Performance Settings

#### Environment Variables
```bash
# Motor optimization settings
export MOTOR_UPDATE_INTERVAL=10    # 10ms update interval
export MOTOR_SYNC_TOLERANCE=5      # 5% sync tolerance
export MOTOR_MAX_POWER=100.0       # 100W max total power
export MOTOR_ACCELERATION_RATE=200 # PWM units per second
export MOTOR_DECELERATION_RATE=300 # PWM units per second
```

## 📷 Camera Optimizations

### 1. Camera Configuration

#### High-Performance Camera Setup
```python
from hardware.optimization.camera_optimizer import camera_optimizer, CameraConfig, CameraBackend

# Configure camera for optimal performance
camera_config = CameraConfig(
    width=640,
    height=480,
    fps=30,
    backend=CameraBackend.PICAMERA2,
    format="RGB888",
    quality=85,
    buffer_size=3,
    auto_exposure=True,
    auto_white_balance=True,
    brightness=0.5,
    contrast=1.0,
    saturation=1.0
)

# Configure and start camera
camera_optimizer.configure(camera_config)
camera_optimizer.start()
```

#### Image Processing Pipeline
```python
# Add optimized image processors
def resize_processor(frame):
    return cv2.resize(frame, (320, 240))

def enhance_processor(frame):
    # Apply CLAHE for better contrast
    lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    lab[:,:,0] = clahe.apply(lab[:,:,0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)

camera_optimizer.add_processor(resize_processor)
camera_optimizer.add_processor(enhance_processor)
```

### 2. Camera Performance Settings

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

# Camera performance
camera_boost=1
```

#### Environment Variables
```bash
# Camera optimization settings
export CAMERA_BACKEND=picamera2
export CAMERA_WIDTH=640
export CAMERA_HEIGHT=480
export CAMERA_FPS=30
export CAMERA_BUFFER_SIZE=3
export CAMERA_QUALITY=85
export CAMERA_AUTO_EXPOSURE=true
export CAMERA_AUTO_WHITE_BALANCE=true
```

## 📊 Sensor Optimizations

### 1. Ultrasonic Sensor

#### High-Performance Ultrasonic Setup
```python
from hardware.optimization.hardware_performance_monitor import hardware_monitor, HardwareComponent

# Monitor ultrasonic sensor performance
@hardware_monitor.monitor_hardware_operation(HardwareComponent.ULTRASONIC, "distance_reading")
def optimized_ultrasonic_reading():
    # Optimized ultrasonic reading with caching
    pass
```

#### Environment Variables
```bash
# Ultrasonic optimization settings
export ULTRA_MEASURE_INTERVAL=50    # 50ms measurement interval
export ULTRA_WRITE_TIMEOUT=100      # 100ms write timeout
export ULTRA_LOG_EVERY=20           # Log every 20 readings
export ULTRA_LOG_DELTA_CM=5         # Log if change > 5cm
```

### 2. Line Tracker

#### High-Performance Line Tracker Setup
```bash
# Line tracker optimization settings
export RATE_HZ=20          # Increased from 10Hz
export KEEPALIVE_SEC=1.0   # Reduced from 2.0s
export GPIO_POLL_INTERVAL=10
export GPIO_DEBOUNCE_MS=5
```

## 🔍 Performance Monitoring

### 1. Hardware Performance Monitoring

#### Start Performance Monitoring
```python
from hardware.optimization.hardware_performance_monitor import hardware_monitor

# Start hardware performance monitoring
await hardware_monitor.start_monitoring(interval=1.0)

# Get performance report
report = hardware_monitor.get_performance_report()
print(f"Performance scores: {report['performance_scores']}")
```

#### Monitor Specific Components
```python
# Monitor motor performance
motor_stats = hardware_monitor.get_component_stats(HardwareComponent.MOTOR)
print(f"Motor performance: {motor_stats}")

# Monitor camera performance
camera_stats = hardware_monitor.get_component_stats(HardwareComponent.CAMERA)
print(f"Camera performance: {camera_stats}")
```

### 2. Performance Alerts

#### Set Performance Thresholds
```python
# Set custom performance thresholds
hardware_monitor.thresholds[HardwareComponent.MOTOR] = {
    "max_duration": 0.05,  # 50ms max duration
    "max_error_rate": 0.01  # 1% max error rate
}
```

## ⚡ Real-Time Optimizations

### 1. Real-Time Scheduling

#### Enable Real-Time Priority
```bash
# Add to /etc/sysctl.conf
kernel.sched_rt_runtime_us = -1
kernel.sched_rt_period_us = 1000000

# Set real-time priority for robot processes
sudo chrt -f -p 99 $(pgrep -f "movement_ws_server.py")
sudo chrt -f -p 99 $(pgrep -f "ultrasonic_sensor_runner.py")
```

### 2. Process Priority

#### Set Process Priorities
```bash
# High priority for critical processes
sudo nice -n -10 python3 movement_ws_server.py
sudo nice -n -10 python3 ultrasonic_sensor_runner.py

# Lower priority for non-critical processes
sudo nice -n 10 python3 video_server.py
```

## 🔋 Power Management

### 1. Power Optimization

#### Dynamic Power Scaling
```python
from hardware.optimization.motor_optimizer import motor_optimizer

# Monitor power consumption
stats = motor_optimizer.get_performance_stats()
power_utilization = stats["power_utilization"]

if power_utilization > 80:
    # Reduce motor speeds to save power
    motor_optimizer.set_motor_speed("front_left", 1500)
    motor_optimizer.set_motor_speed("front_right", 1500)
```

#### Temperature-Based Throttling
```python
# Monitor motor temperatures
for motor_id, state in motor_optimizer.get_all_motor_states().items():
    if state.temperature > 70:  # 70°C threshold
        # Reduce motor speed to prevent overheating
        motor_optimizer.set_motor_speed(motor_id, state.current_speed * 0.8)
```

### 2. Power Settings

#### Environment Variables
```bash
# Power management settings
export MAX_TOTAL_POWER=100.0        # 100W max total power
export TEMPERATURE_THRESHOLD=70     # 70°C temperature threshold
export POWER_THROTTLE_THRESHOLD=80  # 80% power utilization threshold
```

## 📈 Performance Benchmarks

### 1. Expected Performance Improvements

| Optimization | Before | After | Improvement |
|--------------|---------|--------|-------------|
| **GPIO Response Time** | 5-10ms | <1ms | **90% faster** |
| **Motor Response Time** | 50-100ms | <10ms | **90% faster** |
| **Camera Frame Rate** | 15-20 FPS | 30+ FPS | **50% faster** |
| **Sensor Reading** | 10-20ms | <5ms | **75% faster** |
| **System CPU Usage** | 90-100% | <80% | **20% reduction** |

### 2. Performance Testing

#### Run Performance Tests
```bash
# Test GPIO performance
python3 -m hardware.optimization.test_gpio_performance

# Test motor performance
python3 -m hardware.optimization.test_motor_performance

# Test camera performance
python3 -m hardware.optimization.test_camera_performance
```

## 🛠️ Troubleshooting

### 1. Common Issues

#### High CPU Usage
```bash
# Check CPU usage
htop

# Identify high CPU processes
ps aux --sort=-%cpu | head -10

# Check for CPU frequency scaling issues
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

#### Memory Issues
```bash
# Check memory usage
free -h

# Check for memory leaks
cat /proc/meminfo

# Check GPU memory
vcgencmd get_mem gpu
```

#### GPIO Issues
```bash
# Check GPIO permissions
ls -la /dev/gpiochip*

# Check GPIO state
gpio readall

# Test GPIO functionality
python3 -c "import RPi.GPIO as GPIO; print('GPIO OK')"
```

### 2. Performance Debugging

#### Enable Debug Logging
```bash
# Enable hardware performance logging
export HARDWARE_DEBUG=true
export GPIO_DEBUG=true
export MOTOR_DEBUG=true
export CAMERA_DEBUG=true
```

#### Monitor Performance Metrics
```python
# Get real-time performance metrics
from hardware.optimization.hardware_performance_monitor import hardware_monitor

report = hardware_monitor.get_performance_report()
print(f"Hardware performance: {report}")

# Monitor specific components
motor_stats = hardware_monitor.get_component_stats(HardwareComponent.MOTOR)
camera_stats = hardware_monitor.get_component_stats(HardwareComponent.CAMERA)
```

## 🎯 Best Practices

### 1. Hardware Optimization Checklist

- [ ] **CPU Governor**: Set to performance mode
- [ ] **Memory Split**: Optimize GPU memory allocation
- [ ] **GPIO Configuration**: Use hardware PWM and interrupts
- [ ] **Motor Control**: Implement smooth acceleration/deceleration
- [ ] **Camera Setup**: Optimize for target frame rate
- [ ] **Sensor Reading**: Use interrupt-driven reading
- [ ] **Process Priority**: Set real-time priorities
- [ ] **Power Management**: Implement dynamic power scaling
- [ ] **Performance Monitoring**: Enable real-time monitoring
- [ ] **Error Handling**: Implement graceful degradation

### 2. Maintenance

#### Regular Performance Checks
```bash
# Weekly performance check
python3 -m hardware.optimization.performance_check

# Monthly hardware health check
python3 -m hardware.optimization.hardware_health_check
```

#### Performance Optimization Updates
```bash
# Update hardware optimizations
git pull origin master
python3 -m hardware.optimization.update_optimizations
```

## 📚 Additional Resources

- [Raspberry Pi Performance Tuning](https://www.raspberrypi.org/documentation/configuration/)
- [GPIO Performance Optimization](https://gpiozero.readthedocs.io/)
- [Camera Performance Tuning](https://picamera.readthedocs.io/)
- [Motor Control Optimization](https://www.raspberrypi.org/forums/viewtopic.php?t=176241)

---

**Note**: Always test optimizations in a safe environment before deploying to production. Some optimizations may affect system stability if not properly configured.
