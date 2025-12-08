/**
 * Configuration Validators
 * 
 * Client-side validation helpers for configuration values.
 */

export interface ValidationResult {
  valid: boolean;
  errors: string[];
}

/**
 * Validate robot name
 */
export function validateRobotName(name: string): ValidationResult {
  const errors: string[] = [];

  if (!name || name.trim().length === 0) {
    errors.push('Robot name is required');
  }

  if (name.length > 50) {
    errors.push('Robot name must be 50 characters or less');
  }

  if (!/^[a-zA-Z0-9-_ ]+$/.test(name)) {
    errors.push('Robot name can only contain letters, numbers, spaces, hyphens, and underscores');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate network mode
 */
export function validateNetworkMode(mode: string): ValidationResult {
  const errors: string[] = [];

  if (!['ap', 'client'].includes(mode)) {
    errors.push('Network mode must be "ap" or "client"');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate camera backend
 */
export function validateCameraBackend(backend: string): ValidationResult {
  const errors: string[] = [];

  if (!['picamera2', 'v4l2', 'auto', 'mock'].includes(backend)) {
    errors.push('Camera backend must be one of: picamera2, v4l2, auto, mock');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate camera resolution
 */
export function validateCameraResolution(
  width: number,
  height: number
): ValidationResult {
  const errors: string[] = [];

  if (width < 160 || width > 3840) {
    errors.push('Camera width must be between 160 and 3840');
  }

  if (height < 120 || height > 2160) {
    errors.push('Camera height must be between 120 and 2160');
  }

  if (width % 2 !== 0 || height % 2 !== 0) {
    errors.push('Camera resolution must have even dimensions');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate camera FPS
 */
export function validateCameraFPS(fps: number): ValidationResult {
  const errors: string[] = [];

  if (fps < 1 || fps > 120) {
    errors.push('Camera FPS must be between 1 and 120');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate movement profile
 */
export function validateMovementProfile(profile: string): ValidationResult {
  const errors: string[] = [];

  if (!['smooth', 'aggressive', 'precision'].includes(profile)) {
    errors.push('Movement profile must be one of: smooth, aggressive, precision');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate speed value
 */
export function validateSpeed(speed: number, min: number = 0, max: number = 4095): ValidationResult {
  const errors: string[] = [];

  if (speed < min || speed > max) {
    errors.push(`Speed must be between ${min} and ${max}`);
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate brightness
 */
export function validateBrightness(brightness: number): ValidationResult {
  const errors: string[] = [];

  if (brightness < 0 || brightness > 1) {
    errors.push('Brightness must be between 0 and 1');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate IP address
 */
export function validateIPAddress(ip: string): ValidationResult {
  const errors: string[] = [];

  const ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
  if (!ipRegex.test(ip)) {
    errors.push('Invalid IP address format');
    return { valid: false, errors };
  }

  const parts = ip.split('.').map(Number);
  if (parts.some(part => part < 0 || part > 255)) {
    errors.push('IP address octets must be between 0 and 255');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate SSID
 */
export function validateSSID(ssid: string): ValidationResult {
  const errors: string[] = [];

  if (!ssid || ssid.trim().length === 0) {
    errors.push('SSID is required');
  }

  if (ssid.length > 32) {
    errors.push('SSID must be 32 characters or less');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate Wi-Fi password
 */
export function validateWiFiPassword(password: string, minLength: number = 8): ValidationResult {
  const errors: string[] = [];

  if (!password || password.length < minLength) {
    errors.push(`Wi-Fi password must be at least ${minLength} characters`);
  }

  if (password.length > 63) {
    errors.push('Wi-Fi password must be 63 characters or less');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

