/*
# File: tests/unit/utils/envProfile.test.ts
# Tests: getActiveProfile, getEnvByProfile
#
# getActiveProfile — reads NEXT_PUBLIC_NETWORK_PROFILE, validates against whitelist,
#   defaults to 'local' for missing/invalid values.
# getEnvByProfile  — reads profile-keyed env var, falls back to _LOCAL suffix, then ''.
*/

import { getActiveProfile, getEnvByProfile } from '@/utils/envProfile';

// Isolate env between tests
const ORIG_ENV = process.env;

beforeEach(() => {
  process.env = { ...ORIG_ENV };
});

afterAll(() => {
  process.env = ORIG_ENV;
});

// ─────────────────────────────────────────────────────────────
// getActiveProfile
// ─────────────────────────────────────────────────────────────

describe('getActiveProfile', () => {
  it('returns "local" when env var is not set', () => {
    delete process.env.NEXT_PUBLIC_NETWORK_PROFILE;
    expect(getActiveProfile()).toBe('local');
  });

  it('returns "lan" when env var is "lan"', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'lan';
    expect(getActiveProfile()).toBe('lan');
  });

  it('returns "tailscale" when env var is "tailscale"', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'tailscale';
    expect(getActiveProfile()).toBe('tailscale');
  });

  it('returns "local" when env var is "local"', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'local';
    expect(getActiveProfile()).toBe('local');
  });

  it('falls back to "local" for an invalid profile value', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'vpn';
    expect(getActiveProfile()).toBe('local');
  });

  it('falls back to "local" for an empty string', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = '';
    expect(getActiveProfile()).toBe('local');
  });
});

// ─────────────────────────────────────────────────────────────
// getEnvByProfile
// ─────────────────────────────────────────────────────────────

describe('getEnvByProfile', () => {
  it('reads the profile-specific env var when present', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'lan';
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_LAN = 'ws://robot-lan:5000';
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL')).toBe('ws://robot-lan:5000');
  });

  it('falls back to _LOCAL suffix when profile-specific var is absent', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'lan';
    delete process.env.NEXT_PUBLIC_BACKEND_WS_URL_LAN;
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCAL = 'ws://localhost:5000';
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL')).toBe('ws://localhost:5000');
  });

  it('returns empty string when neither profile nor LOCAL var exists', () => {
    process.env.NEXT_PUBLIC_NETWORK_PROFILE = 'lan';
    delete process.env.NEXT_PUBLIC_BACKEND_WS_URL_LAN;
    delete process.env.NEXT_PUBLIC_BACKEND_WS_URL_LOCAL;
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL')).toBe('');
  });

  it('respects an explicit profile override argument', () => {
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_TAILSCALE = 'ws://robot.ts:5000';
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL', 'tailscale')).toBe(
      'ws://robot.ts:5000'
    );
  });

  it('profile argument is case-insensitive', () => {
    process.env.NEXT_PUBLIC_BACKEND_WS_URL_LAN = 'ws://robot-lan:5000';
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL', 'lan')).toBe('ws://robot-lan:5000');
    expect(getEnvByProfile('NEXT_PUBLIC_BACKEND_WS_URL', 'LAN')).toBe('ws://robot-lan:5000');
  });
});
