import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import SensorDashboard from '../src/components/SensorDashboard';
import { act } from 'react'; // Ensure act is imported

beforeAll(() => {
  global.fetch = jest.fn();
});

describe('SensorDashboard Component', () => {
  beforeEach(() => {
    fetch.mockClear();
  });

  it('renders the Line Tracking data', () => {
    render(<SensorDashboard />);
    const lineTrackingElement = screen.getByText(/Line Tracking:/i);
    expect(lineTrackingElement).toBeInTheDocument();
  });

  it('renders the Ultrasonic Distance data', () => {
    render(<SensorDashboard />);
    const ultrasonicDistanceElement = screen.getByText(/Ultrasonic Distance:/i);
    expect(ultrasonicDistanceElement).toBeInTheDocument();
  });

  it('fetches data and updates state', async () => {
    fetch
      .mockImplementationOnce(() =>
        Promise.resolve({
          ok: true,
          json: () => Promise.resolve({ IR01: 1, IR02: 2, IR03: 3 }),
        })
      )
      .mockImplementationOnce(() =>
        Promise.resolve({
          ok: true,
          json: () => Promise.resolve({ distance: 100 }),
        })
      );

    await act(async () => {
      render(<SensorDashboard />);
    });

    await waitFor(() => {
      expect(screen.getByText(/IR01: 1/i)).toBeInTheDocument();
      expect(screen.getByText(/IR02: 2/i)).toBeInTheDocument();
      expect(screen.getByText(/IR03: 3/i)).toBeInTheDocument();
      expect(screen.getByText(/Distance: 100 cm/i)).toBeInTheDocument();
    });
  });
});
