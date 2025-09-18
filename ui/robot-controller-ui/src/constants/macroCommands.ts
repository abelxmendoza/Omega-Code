import { COMMAND } from '@/control_definitions';

export interface MacroCommandOption {
  value: string;
  label: string;
  description?: string;
}

export interface MacroCommandGroup {
  group: string;
  options: MacroCommandOption[];
}

export const MACRO_COMMAND_GROUPS: MacroCommandGroup[] = [
  {
    group: 'Movement',
    options: [
      { value: COMMAND.MOVE_UP, label: 'Move Forward (up)' },
      { value: COMMAND.MOVE_DOWN, label: 'Move Backward (down)' },
      { value: COMMAND.MOVE_LEFT, label: 'Strafe Left' },
      { value: COMMAND.MOVE_RIGHT, label: 'Strafe Right' },
      { value: COMMAND.MOVE_STOP, label: 'Stop (alias)' },
      { value: COMMAND.STOP, label: 'Emergency Stop' },
    ],
  },
  {
    group: 'Speed',
    options: [
      {
        value: COMMAND.SET_SPEED,
        label: 'Set Speed (absolute)',
        description: 'Payload: { "value": 0-4095 }',
      },
      {
        value: COMMAND.INCREASE_SPEED,
        label: 'Increase Speed',
        description: 'Optional payload: { "step": number }',
      },
      {
        value: COMMAND.DECREASE_SPEED,
        label: 'Decrease Speed',
        description: 'Optional payload: { "step": number }',
      },
    ],
  },
  {
    group: 'Camera & Servo',
    options: [
      {
        value: COMMAND.SERVO_HORIZONTAL,
        label: 'Servo Horizontal (relative)',
        description: 'Payload: { "angle": +/- number }',
      },
      {
        value: COMMAND.SERVO_VERTICAL,
        label: 'Servo Vertical (relative)',
        description: 'Payload: { "angle": +/- number }',
      },
      {
        value: COMMAND.SET_SERVO_POSITION,
        label: 'Set Servo Position',
        description: 'Payload: { "horizontal"?: 0-180, "vertical"?: 0-180 }',
      },
      { value: COMMAND.RESET_SERVO, label: 'Reset Camera Servo' },
      { value: COMMAND.CAMERA_SERVO_LEFT, label: 'Camera Nudge Left' },
      { value: COMMAND.CAMERA_SERVO_RIGHT, label: 'Camera Nudge Right' },
      { value: COMMAND.CAMERA_SERVO_UP, label: 'Camera Nudge Up' },
      { value: COMMAND.CAMERA_SERVO_DOWN, label: 'Camera Nudge Down' },
    ],
  },
  {
    group: 'Lighting',
    options: [
      {
        value: COMMAND.SET_LED,
        label: 'Set LED (legacy)',
        description: 'Payload: { "color": "#RRGGBB" }',
      },
      {
        value: COMMAND.LIGHTING_SET_COLOR,
        label: 'Lighting: Set Color',
        description: 'Payload: { "hex": "#RRGGBB" }',
      },
      {
        value: COMMAND.LIGHTING_SET_MODE,
        label: 'Lighting: Set Mode',
        description: 'Payload: { "mode": "single" | "multi" | "two" }',
      },
      {
        value: COMMAND.LIGHTING_SET_PATTERN,
        label: 'Lighting: Set Pattern',
        description: 'Payload: { "pattern": string }',
      },
      {
        value: COMMAND.LIGHTING_SET_INTERVAL,
        label: 'Lighting: Set Interval',
        description: 'Payload: { "ms": number }',
      },
      { value: COMMAND.LIGHTING_TOGGLE, label: 'Lighting: Toggle On/Off' },
    ],
  },
  {
    group: 'Buzzer',
    options: [
      { value: COMMAND.CMD_BUZZER, label: 'Buzzer On' },
      { value: COMMAND.CMD_BUZZER_STOP, label: 'Buzzer Off' },
    ],
  },
  {
    group: 'Status / Misc',
    options: [
      { value: COMMAND.STATUS, label: 'Request Status Snapshot' },
    ],
  },
];

const labelLookup = new Map<string, string>();
for (const group of MACRO_COMMAND_GROUPS) {
  for (const option of group.options) {
    if (!labelLookup.has(option.value)) {
      labelLookup.set(option.value, option.label);
    }
  }
}

export const DEFAULT_MACRO_COMMAND = COMMAND.MOVE_UP;

export const getMacroCommandLabel = (command: string): string =>
  labelLookup.get(command) ?? command;

