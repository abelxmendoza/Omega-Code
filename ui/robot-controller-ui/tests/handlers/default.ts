// tests/handlers/default.ts
import { http, HttpResponse } from 'msw';

export const handlers = [
  // System health — used by systemHealth singleton + SystemHealthContext
  http.get('/api/health', () =>
    HttpResponse.json({ ok: true, status: 'healthy' }, { status: 200 })
  ),

  // Keeps Header/network pills predictable in tests
  http.get('/api/net/summary', () =>
    HttpResponse.json(
      {
        movement: 'ok',
        ultra: 'ok',
        line: 'degraded',
        light: 'ok',
        video: 'unavailable',
      },
      { status: 200 }
    )
  ),

  // Camera health OK by default (override to error in targeted tests)
  http.get('/api/video-health', () =>
    HttpResponse.json({ status: 'ok', latencyMs: 123 }, { status: 200 })
  ),
];
