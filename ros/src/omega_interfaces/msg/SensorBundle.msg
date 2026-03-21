# Aggregate sensor snapshot for the web UI
# Published on /omega/sensor_bundle at 10 Hz by sensor_node
std_msgs/Header header

# Ultrasonic (metres, -1.0 = no reading)
float32 ultrasonic_range_m

# Line tracking (True = sensor detects line)
bool line_left
bool line_center
bool line_right

# Battery
float32 battery_voltage
float32 battery_percentage   # 0.0 - 1.0

# System mode
bool sim_mode
