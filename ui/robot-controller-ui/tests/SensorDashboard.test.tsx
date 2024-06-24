import React from 'react';
import { render, screen } from '@testing-library/react';
import SensorDashboard from '../src/components/SensorDashboard';

describe('SensorDashboard Component', () => {
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
});
