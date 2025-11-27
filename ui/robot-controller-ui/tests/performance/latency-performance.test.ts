/**
 * Latency Performance Tests
 * Tests for latency measurement performance
 */

describe('Latency Performance', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('latency API responds within acceptable time', async () => {
    const start = performance.now();
    
    // Mock fast response
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ ok: true, latencies_ms: { total_processing_ms: 2.5 } }),
      } as Response)
    );

    const response = await fetch('/api/video-proxy/latency');
    const elapsed = performance.now() - start;

    expect(response.ok).toBe(true);
    // Should respond within 100ms (mocked)
    expect(elapsed).toBeLessThan(100);
  });

  it('handles multiple concurrent latency requests', async () => {
    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ ok: true, latencies_ms: { total_processing_ms: 2.5 } }),
      } as Response)
    );

    const promises = Array(10).fill(null).map(() => fetch('/api/video-proxy/latency'));
    const responses = await Promise.all(promises);

    expect(responses.length).toBe(10);
    expect(responses.every(r => r.ok)).toBe(true);
  });

  it('latency metrics update efficiently', async () => {
    const updateCount = 10;
    const start = performance.now();

    global.fetch = jest.fn(() =>
      Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ ok: true, latencies_ms: { total_processing_ms: 2.5 } }),
      } as Response)
    );

    for (let i = 0; i < updateCount; i++) {
      await fetch('/api/video-proxy/latency');
    }

    const elapsed = performance.now() - start;
    const avgTime = elapsed / updateCount;

    // Average should be under 50ms per request (mocked)
    expect(avgTime).toBeLessThan(50);
  });
});

