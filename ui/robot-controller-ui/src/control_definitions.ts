/*
# File: /Omega-Code/ui/robot-controller-ui/src/control_definitions.ts
# Summary:
Centralized command strings + lighting constants used across the UI.
- Matches movement_ws_server.py (set-speed, stop, move-*, servo-*, buzz, buzz-stop, camera-servo-*)
- Provides stable names for UI code
- Includes compatibility aliases (CMD_SERVO_*), so legacy imports won’t break
- Adds a few RESERVED commands for future automation / video servoing (backend TODO)

Tips:
- Use COMMAND.SET_SPEED with payload { value: 0..4095 }.
- Movement commands accept optional { speed?: 0..4095, durationMs?: number }.
*/

export const COMMAND = {
  // --- Motors ---------------------------------------------------------------
  CMD_MOTOR: 'CMD_MOTOR',                 // (legacy placeholder; not used by server)
  MOVE_UP: 'move-up',                     // { speed?, durationMs? }
  MOVE_DOWN: 'move-down',                 // { speed?, durationMs? }
  MOVE_LEFT: 'move-left',                 // { speed?, durationMs? }
  MOVE_RIGHT: 'move-right',               // { speed?, durationMs? }
  MOVE_STOP: 'move-stop',                 // alias the server normalizes to "stop"
  STOP: 'stop',                           // immediate halt

  // Speed (server expects PWM 0..4095 in "value")
  SET_SPEED: 'set-speed',                 // { value: 0..4095 }
  INCREASE_SPEED: 'increase-speed',       // increments on server (DEFAULT_SPEED_STEP)
  DECREASE_SPEED: 'decrease-speed',       // decrements on server

  // --- Servos ---------------------------------------------------------------
  // Preferred names (match server):
  SERVO_HORIZONTAL: 'servo-horizontal',   // { angle: ±N } relative
  SERVO_VERTICAL: 'servo-vertical',       // { angle: ±N } relative
  SET_SERVO_POSITION: 'set-servo-position', // { horizontal?: 0..180, vertical?: 0..180 }
  RESET_SERVO: 'reset-servo',

  // Compatibility aliases (old UI code may reference these):
  CMD_SERVO_HORIZONTAL: 'servo-horizontal',
  CMD_SERVO_VERTICAL: 'servo-vertical',

  // Camera nudge aliases (use DEFAULT_SERVO_STEP on server)
  CAMERA_SERVO_LEFT: 'camera-servo-left',
  CAMERA_SERVO_RIGHT: 'camera-servo-right',
  CAMERA_SERVO_UP: 'camera-servo-up',
  CAMERA_SERVO_DOWN: 'camera-servo-down',

  // --- LEDs / Lighting (UI-side; wire to your lighting backend if present) ---
  CMD_LED: 'CMD_LED',
  CMD_LED_MOD: 'CMD_LED_MOD',
  SET_LED: 'set-led',
  LED_PATTERN: 'led-pattern',
  LED_TIMING: 'led-timing',
  LED_BRIGHTNESS: 'led-brightness',
  LIGHTING_SET_COLOR: 'lighting-set-color',
  LIGHTING_SET_MODE: 'lighting-set-mode',
  LIGHTING_SET_PATTERN: 'lighting-set-pattern',
  LIGHTING_SET_INTERVAL: 'lighting-set-interval',
  LIGHTING_TOGGLE: 'lighting-toggle',

  // --- Buzzer ---------------------------------------------------------------
  CMD_BUZZER: 'buzz',                     // ON until "buzz-stop" (great for hold-to-horn)
  CMD_BUZZER_STOP: 'buzz-stop',           // OFF
  // RESERVED (backend TODO): 'buzz-duration' to support timed beeps from a single message.
  // BUZZ_DURATION: 'buzz-duration',      // e.g. { ms: 250 }

  // --- Sensors / Power / Status --------------------------------------------
  CMD_SONIC: 'CMD_SONIC',
  CMD_LIGHT: 'CMD_LIGHT',
  CMD_POWER: 'CMD_POWER',
  CMD_MODE: 'CMD_MODE',
  STATUS: 'status',                       // ask server for current speed/servo state

  // --- Automation / Video servoing (RESERVED: add handlers in backend) ------
  // Useful names to standardize now; implement later server-side:
  // AUTO_TRACK_ON: 'camera-track-start',  // start vision-based tracking loop
  // AUTO_TRACK_OFF: 'camera-track-stop',  // stop tracking
  // RUN_MACRO: 'run-macro',               // { name: string, args?: any }
  // STOP_MACRO: 'stop-macro',
} as const;

// Lighting Pattern Constants - Optimized with cool patterns
export const LIGHTING_PATTERNS = [
  'static',    // steady color
  'blink',     // on/off at defined interval
  'fade',      // smooth transitions
  'chase',     // chasing effect
  'rainbow',   // spectrum sweep
  'music',     // audio reactive pulse
  'lightshow', // multi-stage animated showpiece
  'rave',      // energetic dancing lights (no audio required)
  'breathing', // smooth breathing pulse (energy-efficient)
  'aurora',    // flowing northern lights effect
  'matrix',    // Matrix-style rain effect
  'fire',      // flickering fire effect
] as const;

// Lighting Mode Constants
export const LIGHTING_MODES = [
  'single',   // one color for all LEDs
  'multi',    // multiple colors simultaneously
  'two',      // alternate between two colors
] as const;

// Optional helper types if you want them elsewhere in the UI:
export type CommandKey = keyof typeof COMMAND;
export type LightingPattern = typeof LIGHTING_PATTERNS[number];
export type LightingMode = typeof LIGHTING_MODES[number];
