# Recorded perception reports

Omega-Code can inspect version 1 `bench_replay_report` JSON produced by
[`Jetson-RealSense-Perception`](https://github.com/abelxmendoza/Jetson-RealSense-Perception).
The producer repository remains the source of truth for depth policy decisions.

The adapter at
`servers/robot_controller_backend/perception/bench_report_adapter.py` validates
the envelope, schema version, event order, declared counts, source label, and
PASS/FAIL/SKIP outcomes. It summarizes recorded events for development and
verification on a Mac or Linux machine.

Example:

```python
from perception import load_bench_report

summary = load_bench_report("/path/to/bench-report.json")
print(summary.latest_status)
print(summary.status_counts)
print(summary.outcome_counts)
```

## Safety boundary

This is deliberately an offline, read-only adapter. It does not import GPIO,
ROS, motor, movement, or WebSocket modules. It does not publish velocity or
vehicle commands. `STOP` and `SENSOR_FAULT` are both counted as blocking events,
but turning that recorded information into a live rover gate requires a separate
hardware-reviewed integration after Omega-1 is repaired.

Synthetic recordings are not hardware evidence. Consumers must use the report's
explicit source value and preserve SKIP separately from PASS.

Run the focused tests from the backend directory:

```bash
cd servers/robot_controller_backend
python3 -m pytest tests/unit/test_bench_report_adapter.py -q
```
