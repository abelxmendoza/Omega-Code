# Omega1 Gazebo model — calibration pending

This is an original parameterized engineering approximation of Omega1's ordinary-wheel
Freenove FNK0043 hardware, not a measured digital twin. It stays optional: existing
Pi software and lightweight SIL mode are unchanged. No RealmForge or perception
checkout is required. External RealSense is deliberately not mounted on the model.

## Generate and run

Python 3.10+ generates all assets using only the standard library. Gazebo Harmonic
(`gz sim` / Sim 8) is required to execute the world; OpenSCAD is optional for CAD.
From the Omega-Code repository root:

```bash
python3 simulation/omega1/generate.py
bash simulation/omega1/run.sh
# In another terminal: slow forward movement, then stop
python3 simulation/omega1/control.py --linear 0.10 --seconds 1
python3 simulation/omega1/control.py --pan 0.2 --tilt 0.1
# Optional ROS bridge, with ros_gz_bridge installed for your ROS/Gazebo pairing
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/simulation/omega1/bridge.yaml
```

The bridge uses simulation command/odom topics, not the existing physical motor
node's input. Do not launch the existing fake motor/sensor publishers alongside
Gazebo. Existing autonomy/API nodes are NOT wired into this new mode yet.
`control.py` is a separate finite-duration command path, never a GPIO driver.
It sends zero velocity on normal completion/interrupt. Its watchdog only works
while the adapter is running and ticked; a killed process or transport failure
can leave Gazebo's last command active. It is not a simulator-side watchdog.

## Model and simulated hardware inventory

| Component | Implemented representation | Fidelity / remaining work |
|---|---|---|
| Chassis, board, battery holder, Pi, motors | Original solid geometry, mass lumped into base | Dimensions, COM, inertia and placement estimated; no PCB traces or electronics CAD |
| Four ordinary tires and yellow hubs | Four revolute joints, collision cylinders, visual rotation marks | Tire grip, motor torque, slip and turning need calibration; ideal DiffDrive actuation |
| Camera pan/tilt | Two limited joints and position controllers | Conservative angles; measure PWM-to-angle relationship and latency |
| CSI camera | 640x480 RGB at 15 Hz on moving head | Intrinsics, distortion, exposure and latency uncalibrated |
| HC-SR04-like sensor | Nine-ray GPU lidar fan, 10 Hz, separate sim topic | Optical ray proxy; not ultrasonic acoustics or a ROS Range sensor; no echo/dropout/noise model yet |
| Line board | Geometry only | Reflectance simulation not implemented |
| Odometry | Gazebo DiffDrive output | Ideal joint feedback, not physical encoders; physical Omega1 has no encoders |
| PCA9685 / GPIO / power | Not emulated | No register-level I2C, Raspberry Pi execution, current draw, battery sag, LED or buzzer simulation |

Generated `omega1.stl` is a static millimetre-scale assembly snapshot, not articulated.
`python3 simulation/omega1/preview.py` optionally renders an inspection image
(requires matplotlib/numpy). Regenerate STL using OpenSCAD after changing parameters.

Generated `omega1.scad` is editable assembly geometry in millimetres. It is not
fabrication-ready CAD. Generated SDF is the authoritative articulated model.
Edit `parameters.json` then regenerate; do not hand-edit generated assets.
Current overall dimensions are estimates; existing ROS defaults (0.05 m wheel
radius, 0.20 m separation) were not treated as measurements.

## Verification

```bash
python3 -m unittest discover -s simulation/omega1/tests -v
# On a machine with Gazebo and SDF tools installed:
gz sdf -k simulation/omega1/generated/models/omega1/model.sdf
GZ_SIM_RESOURCE_PATH="$(pwd)/simulation/omega1/generated/models" gz sdf -k simulation/omega1/generated/bench.sdf
bash simulation/omega1/run.sh -s --iterations 1000
```

Cloud checks: six Python tests and shell syntax pass; generated CAD renders/exports
are checked separately. Gazebo is unavailable here. SDF semantics, physics,
wheel motion, servo motion, camera images and range output must still be tested
in Gazebo. No autonomous obstacle-navigation result has been demonstrated.

Acceptance on Gazebo: (1) rests on all four wheels without exploding/drifting,
(2) positive X command moves forward and positive yaw turns left, (3) stop halts
motion, (4) pan/tilt moves head without breaking joints, (5) RGB/range topics
publish and the obstacle is observed, (6) collision prevents driving through it.
Then connect Omega's controller and measure a goal-around-obstacle test.

## Calibration checklist for an accurate replica

Measure wheel diameter/width, wheel-centre track width, front/rear axle spacing,
board outline, ground clearance, total mass, head and sensor offsets. Confirm
board revision and battery configuration. Record forward/reverse speed, turning
radius, stop distance, motor deadband, backlash and servo angle versus command.
Calibrate camera intrinsics and ultrasonic range/noise/dropouts against targets.
Use repeated real/sim trajectories to report pose, range and clearance error.
Accuracy claims must include measured tolerances; photos alone cannot supply them.

## Sources and asset provenance

- Physical configuration: repo `README.md` and `image/README/1719168223539.png`.
- Servo limits and no encoders: README hardware fixes/known limitations;
  `servers/robot_controller_backend/controllers/servo_control.py`.
- [Freenove official assembly guide](https://docs.freenove.com/projects/fnk0043/en/latest/fnk0043/codes/tutorial/2_Assemble_Smart_Car.html).
- [Freenove official repository](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi),
  inspected commit `a49db4b9dfa9b7a82d172354cb3b4e0ed64985e5`.
  No STEP, STL, URDF, OBJ or KiCad PCB files found in that checkout. Upstream
  CC BY-NC-SA 3.0 applies to its resources; no upstream code/meshes copied here.
- [Gazebo Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
  and [joint position controller](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html).
- Amazon search did not establish reliable dimensions for this exact assembled
  revision; no listing dimensions are used as robot measurements.
