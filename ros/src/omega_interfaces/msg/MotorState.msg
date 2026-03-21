# Omega-1 motor controller state
# Published on /omega/motor_state at 10 Hz by motor_controller_node
std_msgs/Header header

# Actual PWM values being applied to the hardware (-4095 to 4095)
int32 left_pwm
int32 right_pwm

# Target PWM values (before ramp is applied)
int32 target_left_pwm
int32 target_right_pwm

# Estimated velocities from dead-reckoning odometry
float32 linear_velocity    # m/s
float32 angular_velocity   # rad/s

# Watchdog state: "active" | "triggered" | "disabled"
string watchdog_state
float32 watchdog_elapsed   # seconds since last /cmd_vel received

# Safety flags
bool thermal_throttled
bool thermal_killed
bool sim_mode
