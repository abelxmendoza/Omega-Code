import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { setupWsHarness } from './helpers/wsHarness';
import SensorDashboard from '@/components/SensorDashboard'; // adjust if different

const ws = setupWsHarness();

describe('SensorDashboard (WS integration)', () => {
  it('reacts to incoming sensor data', async () => {
    render(<SensorDashboard />);
    await waitFor(() => expect(ws.getSocket()).toBeTruthy());

    await act(async () => {
      ws.send({ type: 'sensors', ultrasonic: 42, line: 'left', light: 0.73 });
    });

    // Use tolerant matchers; tweak to your actual UI text/labels
    expect(
      screen.getByText((_, el) => el?.textContent?.match(/\b42\b/) != null)
    ).toBeInTheDocument();
  });
});
