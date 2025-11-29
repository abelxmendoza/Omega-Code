import os
import shutil
import re
from pathlib import Path

# Define the paths and the new directory structure
# Determine project root and backend directory
PROJECT_ROOT = Path(os.environ.get("OMEGA_CODE_ROOT", Path(__file__).resolve().parents[1]))
base_path = PROJECT_ROOT / "servers" / "robot_controller_backend"
base_path = str(base_path)
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

# Create new directories
for folder in new_structure.keys():
    new_folder_path = os.path.join(base_path, folder)
    os.makedirs(new_folder_path, exist_ok=True)

# Move files to new directories
for folder, files in new_structure.items():
    for file_name in files:
        src_path = os.path.join(base_path, file_name)
        if os.path.exists(src_path):
            dst_path = os.path.join(base_path, folder, file_name)
            shutil.move(src_path, dst_path)

print("Files moved successfully.")

# Function to update import paths in Go files
def update_go_imports(file_path, old_folder, new_folder):
    with open(file_path, "r") as file:
        content = file.read()

    updated_content = content.replace(f'/{old_folder}/', f'/{new_folder}/')

    with open(file_path, "w") as file:
        file.write(updated_content)

# Function to update import paths in Python files
def update_python_imports(file_path):
    with open(file_path, "r") as file:
        content = file.read()

    # Update the imports based on the new structure
    for folder in new_structure.keys():
        content = re.sub(f'from {folder} import', f'from {folder} import', content)
        content = re.sub(f'import {folder}', f'import {folder}', content)

    with open(file_path, "w") as file:
        file.write(content)

# Update imports in Go files
go_files = [os.path.join(base_path, "main.go")] + [os.path.join(base_path, folder, f) for folder in new_structure.keys() for f in os.listdir(os.path.join(base_path, folder)) if f.endswith(".go")]
for go_file in go_files:
    update_go_imports(go_file, "commands", "commands")
    update_go_imports(go_file, "gpio", "gpio")
    update_go_imports(go_file, "sensors", "sensors")
    update_go_imports(go_file, "controllers", "controllers")
    update_go_imports(go_file, "utils", "utils")

# Update imports in Python files
python_files = [os.path.join(base_path, folder, f) for folder in new_structure.keys() for f in os.listdir(os.path.join(base_path, folder)) if f.endswith(".py")]
for python_file in python_files:
    update_python_imports(python_file)

print("Imports updated successfully.")
