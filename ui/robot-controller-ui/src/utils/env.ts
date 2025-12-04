/**
 * Environment Detection Utilities
 * Detects if we are running on Vercel (cloud) vs local development
 */

// Detect if we are on Vercel (production build)
export const IS_CLOUD = !!process.env.VERCEL_ENV;

export const IS_LOCAL = !IS_CLOUD;

// Robot backend should only run locally
export const ROBOT_ENABLED = IS_LOCAL;

// Provide a safe dummy base URL for production
export const ROBOT_BASE_URL = IS_LOCAL
  ? "http://omega1.local:8080"
  : "http://localhost:0"; // invalid on purpose to prevent accidental calls

