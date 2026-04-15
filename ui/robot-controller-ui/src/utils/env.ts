/**
 * Environment Detection Utilities
 * Detects if we are running on Vercel (cloud) vs local development.
 *
 * Problem solved: process.env.VERCEL / VERCEL_ENV are set in the Node.js
 * build/SSR environment but are NOT embedded into the client-side bundle
 * (only NEXT_PUBLIC_* vars are).  Using bare VERCEL_ENV causes a server↔client
 * mismatch that triggers React hydration errors #418 / #423.
 *
 * Solution: detect cloud at runtime using window.location.hostname on the
 * client, and fall back to process.env.VERCEL on the server.  The two checks
 * produce consistent results for every deployment scenario:
 *   - Vercel:     server VERCEL=1 → IS_CLOUD=true  /  client *.vercel.app → IS_CLOUD=true  ✓
 *   - Local dev:  server VERCEL unset → IS_CLOUD=false / client localhost → IS_CLOUD=false  ✓
 */

const _KNOWN_LOCAL_PREFIXES = [
  'localhost',
  '127.0.0.1',
  '192.168.',
  '10.',
  '172.',
  'omegaone',
  'omega1',
];

function _detectCloud(): boolean {
  if (typeof window === 'undefined') {
    // Server-side (SSR / build time) — use Node env var set by Vercel
    return !!process.env.VERCEL;
  }
  // Client-side — check hostname so the result matches the SSR value
  const host = window.location.hostname;
  return !_KNOWN_LOCAL_PREFIXES.some(
    (prefix) => host === prefix || host.startsWith(prefix)
  );
}

// Evaluated once at module-init time — consistent across SSR and hydration
// as long as the deployment scenario is either "Vercel" or "local network".
export const IS_CLOUD = _detectCloud();

export const IS_LOCAL = !IS_CLOUD;

// Robot backend should only be contacted when running on a local network host.
export const ROBOT_ENABLED = IS_LOCAL;

// Safe dummy base URL for production (intentionally unreachable)
export const ROBOT_BASE_URL = IS_LOCAL
  ? "http://omegaone:8000"
  : "http://localhost:0";
