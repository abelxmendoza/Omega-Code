import os
import re

# Define the paths and the new directory structure
base_path = "/Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend"
new_structure = {
    "commands": ["command_definitions.py", "command_processor.go"],
    "gpio": ["gpio_real.go", "gpio_mock.go", "gpio_simulator.py"],
    "sensors": ["adc.py", "ultrasonic_sensor.go", "ultrasonic_sensor.py"],
    "controllers": ["motor_control.py", "servo_control.py", "lineTracking.go", "line_tracking.py"],
    "video": ["video_server.py"],
    "utils": ["threading_control.py", "pca9685.py", "mock_pca9685.py", "led_control.py"],
    "tests": ["main_test.go", "led_control_test.py", "mock_pca9685_test.py", "servo_control_test.py"],
    "certs": [],
    "rust_module": [],
}

# Function to update import paths in Python files
def update_python_imports(file_path):
    with open(file_path, "r") as file:
        content = file.read()

    # Update the imports based on the new structure
    for folder in new_structure.keys():
        for file_name in new_structure[folder]:
            module_name = os.path.splitext(file_name)[0]  # Remove file extension
            if file_name.endswith(".py"):
                pattern = f'(from|import) {module_name}'
                replacement = f'\\1 {folder}.{module_name}'
                content = re.sub(pattern, replacement, content)

    with open(file_path, "w") as file:
        file.write(content)

# Update imports in test files
test_files = [os.path.join(base_path, "tests", f) for f in os.listdir(os.path.join(base_path, "tests")) if f.endswith(".py")]
for test_file in test_files:
    update_python_imports(test_file)

print("Test imports updated successfully.")
