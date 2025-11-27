/**
 * System Mode API Integration Tests
 */

import { buildGatewayUrl } from '@/config/gateway';

describe('System Mode API Integration', () => {
  const API_BASE = '/api/system/mode';

  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('fetches system mode list', async () => {
    const mockModes = {
      ok: true,
      modes: {
        0: { mode: 0, name: 'CAMERA_ONLY', description: 'Camera Only', available: true },
        1: { mode: 1, name: 'MOTION_DETECTION', description: 'Motion Detection', available: true },
      },
      current_mode: 0,
    };

    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve(mockModes),
      } as Response)
    );

    const response = await fetch(`${API_BASE}/list`);
    const data = await response.json();

    expect(data.ok).toBe(true);
    expect(data.modes).toBeDefined();
    expect(data.current_mode).toBe(0);
  });

  it('fetches system mode status', async () => {
    const mockStatus = {
      ok: true,
      mode: 0,
      description: 'Camera Only',
      manual_override: false,
      hybrid_mode: 'pi_only',
      orin_available: false,
    };

    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve(mockStatus),
      } as Response)
    );

    const response = await fetch(`${API_BASE}/status`);
    const data = await response.json();

    expect(data.ok).toBe(true);
    expect(data.mode).toBe(0);
    expect(data.hybrid_mode).toBe('pi_only');
  });

  it('sets system mode', async () => {
    const mockResponse = {
      ok: true,
      message: 'System mode set to 3',
      mode: 3,
    };

    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve(mockResponse),
      } as Response)
    );

    const response = await fetch(`${API_BASE}/set`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: 3 }),
    });
    const data = await response.json();

    expect(data.ok).toBe(true);
    expect(data.mode).toBe(3);
    expect(global.fetch).toHaveBeenCalledWith(
      expect.stringContaining('/set'),
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ mode: 3 }),
      })
    );
  });

  it('handles invalid mode gracefully', async () => {
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: false,
        status: 400,
        json: () => Promise.resolve({ error: 'Invalid mode. Must be 0-7' }),
      } as Response)
    );

    const response = await fetch(`${API_BASE}/set`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: 99 }),
    });
    const data = await response.json();

    expect(data.error).toBeDefined();
  });
});

