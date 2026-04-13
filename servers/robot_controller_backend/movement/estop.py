"""
Shared emergency-stop event for motor-control and autonomy modules.

Any code in the same Python process (movement_ws_server, autonomy modes,
API routes) can import and use ESTOP_EVENT to coordinate an immediate halt
without tight module coupling.

Usage
-----
  from movement.estop import ESTOP_EVENT

  # Trigger e-stop (movement server on "stop" command, API route, etc.):
  ESTOP_EVENT.set()

  # Clear e-stop (next movement command, autonomy restart):
  ESTOP_EVENT.clear()

  # Autonomy loop check:
  if ESTOP_EVENT.is_set():
      bridge.stop()
      ...wait or exit...
"""

import threading

# Single module-level threading.Event importable by any code in the process.
ESTOP_EVENT: threading.Event = threading.Event()
