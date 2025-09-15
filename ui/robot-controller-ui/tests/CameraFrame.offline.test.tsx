/**
 * File: tests/CameraFrame.offline.test.tsx
 * Purpose:
 *   Verify CameraFrame degrades gracefully when /api/video-health fails:
 *   - Shows a visible "Retry" button
 *   - Announces a non-playing state for screen readers
 *
 * Pre-reqs:
 *   - MSW is wired in tests/setupTests.ts and exports `server` from tests/mswServer.ts
 *   - TextEncoder/TextDecoder polyfill is set in setup (see notes below)
 *
 * Error-handling tips:
 *   - If this test hangs on `findByRole('button', {name:/retry/i})`, your UI text may differ.
 *     Update the matcher or add a data-testid to the Retry button.
 *   - If you see "Unhandled request" from MSW, confirm the path is EXACTLY '/api/video-health'
 *     and that setupTests registers default handlers before each test.
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import { http, HttpResponse } from 'msw';
import { server } from './mswServer';
import CameraFrame from '@/components/CameraFrame';

describe('CameraFrame (offline behavior)', () => {
  it('shows Retry when /api/video-health returns 502', async () => {
    // Force the health endpoint to return a shaped failure for THIS test only.
    server.use(
      http.get('/api/video-health', () =>
        HttpResponse.json({ ok: false, code: 'timeout' }, { status: 502 })
      )
    );

    render(<CameraFrame />);

    // Retry button appears in offline state
    const retry = await screen.findByRole('button', { name: /retry/i });
    expect(retry).toBeInTheDocument();

    // Screen-reader status region reflects non-playing state
    const sr = screen.getByText((_, el) => el?.textContent?.match(/video status/i) != null);
    expect(sr.textContent?.toLowerCase()).not.toMatch(/playing/);
  });
});
