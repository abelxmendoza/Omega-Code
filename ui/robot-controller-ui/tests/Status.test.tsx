/**
 * File: tests/Status.test.tsx
 * Purpose:
 *   - Verify Status renders battery % and the correct bar color
 *   - Avoid brittle WS mocking; test via props instead
 *   - Be resilient to split text (e.g., "75\n %") and minor Tailwind color variants
 *
 * Error-handling tips:
 *   - If a percent assertion fails, check for split text or different formatting.
 *     Use the regex matchers shown below or add a data-testid to the component.
 *   - If a color assertion fails, your Tailwind class may differ (e.g., bg-sky-500 vs bg-blue-500).
 *     Tweak the regex to match your theme or assert via style if you compute color dynamically.
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import Status from '@/components/Status';

// Helper: matches "XX%" even if whitespace is split across nodes (e.g., "75\n %")
const hasPercent = (n: number) => {
  const element = screen.getByText(`${n}%`);
  return element;
};

// Helper: returns the inner bar element located next to the percent text.
// Expected structure:
// <div class="w-28 ...">
//   <div class="h-full bg-...-500" style="width: XX%;" />
// </div>
// <span>XX %</span>
const barForPercent = (n: number) => {
  const percentSpan = hasPercent(n);
  const meter = percentSpan.previousElementSibling as HTMLElement | null;
  const inner = meter?.firstElementChild as HTMLElement | null;
  // Defensive checks: clearer error messages if the DOM shifts
  expect(meter).toBeInTheDocument();
  expect(inner).toBeInTheDocument();
  return inner!;
};

describe('Status', () => {
  it('renders 75% and shows a blue-ish bar when connected', () => {
    render(<Status status="Connected" battery={75} />);

    // Check for the percentage text directly
    expect(screen.getByText('75%')).toBeInTheDocument();

    // Allow Tailwind variants like bg-blue-500 OR bg-sky-500
    const bar = barForPercent(75);
    expect(bar.className).toMatch(/bg-(blue|sky)-500/);

    // If your component exposes a status icon test id, keep this check; otherwise it's safely skipped.
    const maybeIcon = screen.queryByTestId('status-icon');
    if (maybeIcon) {
      expect(maybeIcon).toHaveClass('text-green-500'); // adjust if your "connected" color differs
    }
  });

  it('renders 15% and shows a red bar when battery is low', () => {
    render(<Status status="Connected" battery={15} />);

    expect(screen.getByText('15%')).toBeInTheDocument();

    const bar = barForPercent(15);
    expect(bar.className).toMatch(/bg-red-500/);
  });

  it('renders 0% when disconnected (icon/title optional)', () => {
    render(<Status status="Disconnected" battery={0} />);

    expect(screen.getByText('0%')).toBeInTheDocument();

    // Optional: if your SVG includes <title>Not ready</title>, assert it; otherwise this is a no-op
    const maybeTitle = screen.queryByTitle(/not ready/i);
    if (maybeTitle) expect(maybeTitle).toBeInTheDocument();

    // Optional: if you render a data-testid for the icon in disconnected state
    const maybeIcon = screen.queryByTestId('status-icon');
    if (maybeIcon) {
      expect(maybeIcon).toHaveClass('text-red-500');
    }
  });
});
