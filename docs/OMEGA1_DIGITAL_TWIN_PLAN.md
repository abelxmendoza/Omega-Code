# Omega1 Gazebo prototype — phone/cloud handoff

Updated 2026-10-19 (Los Angeles).

## Goal and current evidence

Build a parameterized, animated four-wheel Omega1 model in Omega-Code, independent of RealmForge and the external RealSense scanner. Keep the existing SIL mode. A local original-geometry prototype was generated under /tmp/omega-digital-prototype; three tests passed including Gazebo SDF parsing of model and standalone world. It defines four wheel joints, constrained pan/tilt, schematic Pi board, RGB camera, DiffDrive and joint controllers. Runtime motion, rendering and physical fidelity are unverified. The local source has NOT yet been uploaded to this branch: the library laptop stopped responding during the subsequent save/test step. Recover/review that source before claiming the model is available from GitHub. This branch currently preserves research and next steps only.

## Checked sources

Amazon: https://www.amazon.com/dp/B07YD2LT9D
Listing identifies FNK0043 and four DC geared motors, four wheels, two servo packages, camera, ultrasonic module and line-tracking module. A reliable assembled dimension/mass table was not retrieved. Do not use shipping/package dimensions as robot geometry.

Manufacturer: https://store.freenove.com/products/fnk0043
Assembly: https://docs.freenove.com/projects/fnk0043/en/latest/fnk0043/codes/tutorial/2_Assemble_Smart_Car.html
Original kit: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi
Read Code/Server/motor.py: Ordinary_Car uses PCA9685 address 0x40, 50 Hz and bounded signed PWM duty. No vendor code was copied into the prototype. The official recursive tree contained tutorials/component datasheets, but no full STEP/STL/URDF/KiCad chassis design was found. Vendor README specifies CC BY-NC-SA 3.0; track licenses for any future asset reuse.

Pi 4 mechanical reference: https://pip.raspberrypi.com/categories/559-mechanical
Gazebo multiwheel DiffDrive interface: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html

Omega remote master inspected: 1d7bb60d68520639578917b9fbb2a70cc903dd0a. No CAD/URDF/KiCad design files were found in its Git tree. The desktop working copy has many unrelated modifications; preserve them.

## Fidelity and measurements

All prototype geometry/mass/contact/servo/camera values are illustrative assumptions, NOT calibrated manufacturer values. Candidate defaults include 0.0325 m wheel radius, 0.14 m axle spacing, 0.14 m wheel track, 0.65 kg body mass. They must not be advertised as accurate specifications.

Measure wheel diameter/width, axle spacing, centre-to-centre track, chassis size/clearance, assembled mass, sensor positions, servo axes/limits, battery/board placement. Validate straight motion, turns, stopping distance, camera calibration and sensor latency against hardware. KiCad geometry does not emulate Pi execution, I2C/PCA9685 electronics or current draw.

## Resume steps

1. Recover simulation/gazebo/generate_model.py and tests/test_gazebo_prototype.py from /tmp/omega-digital-prototype on the library laptop if still available; otherwise regenerate from this documented scope.
2. Review and commit the source on this review branch, add CI, validate the model/world with gz sdf -k, and run a manual joint/contact test.
3. Implement simulation-only command/odometry/camera adapters, then ultrasonic/line sensors and fault injection. Do not connect simulated commands to physical motors by default.
4. Later calibrate with actual Omega1 measurements and compare sim-to-real outcomes.

## Notification and execution status

A one-time ChatGPT check-in was scheduled for about five minutes after the phone handoff request. Notification delivery depends on app/OS settings. It is NOT a continuous coding worker or proof of completion. The current local session cannot continue executing on a disconnected laptop. GitHub notes remain available from a phone; do not claim a persistent cloud build exists until one has actually been launched and verified.
