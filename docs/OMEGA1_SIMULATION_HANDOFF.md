# Resume Omega1 simulation here

User goal: a faithful animated 3D replica with simulated hardware, then navigation
in environments exported by Jetson-RealSense-Perception. Keep Omega-Code standalone;
RealmForge is an optional shared/team-owned project. PX4 later consumes compatible
worlds. External RealSense remains separate from Omega1's onboard sensors.

Read `simulation/omega1/README.md` and its fidelity table before continuing.
The model is original procedural SDF/OpenSCAD with estimated geometry. Six focused
tests pass; Gazebo runtime is unverified because this cloud machine has no gz.
Existing SIL/hardware startup and ROS controller paths are unchanged.

Next: run Gazebo acceptance checks, resolve any model/plugin/runtime issues, connect
existing autonomy through a dedicated bridge without duplicate odometry publishers,
and run one obstacle-navigation trial with recorded success and clearance. Measure
the actual robot before calling this an exact replica. Line sensors, electronics,
battery/motor dynamics and ROS Range conversion remain unfinished.
