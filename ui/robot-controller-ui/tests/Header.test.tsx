import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom'; // for the extra matchers like toHaveClass and toHaveTextContent
import Header from '../src/components/Header'; // Ensure this path is correct


describe('Header Component', () => {
  it('renders the application title', () => {
    render(<Header isConnected={true} batteryLevel={75} />);

    const titleElement = screen.getByText(/Robot Controller/i);
    expect(titleElement).toBeInTheDocument();
  });

  it('displays the correct connection status', () => {
    const { rerender } = render(<Header isConnected={true} batteryLevel={75} />);

    const statusElement = screen.getByText(/Status:/i);
    expect(statusElement).toBeInTheDocument();
    expect(screen.getByTestId('status-icon')).toHaveClass('text-green-500');

    rerender(<Header isConnected={false} batteryLevel={75} />);
    expect(screen.getByTestId('status-icon')).toHaveClass('text-red-500');
    expect(statusElement).toHaveTextContent('Status: Disconnected');
  });

  it('displays the correct battery level', () => {
    render(<Header isConnected={true} batteryLevel={75} />);

    const batteryElement = screen.getByText(/Battery:/i);
    expect(batteryElement).toBeInTheDocument();
    expect(batteryElement).toHaveTextContent('Battery: 75%');
  });
});
