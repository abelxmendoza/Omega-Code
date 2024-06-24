import React from 'react';
import { render, screen } from '@testing-library/react';
import Header from '../src/components/Header';

describe('Header Component', () => {
  it('renders the application title', () => {
    render(<Header isConnected={true} batteryLevel={75} />);

    const titleElement = screen.getByText(/Robot Controller/i);
    expect(titleElement).toBeInTheDocument();
  });

  it('displays the correct connection status', () => {
    const { rerender } = render(<Header isConnected={true} batteryLevel={75} />);

    const statusElements = screen.getAllByText((content, element) => element.textContent.includes('Status:'));
    expect(statusElements[0]).toHaveTextContent('Status:');
    expect(screen.getByTestId('status-icon')).toHaveClass('text-green-500');

    rerender(<Header isConnected={false} batteryLevel={75} />);
    expect(screen.getByTestId('status-icon')).toHaveClass('text-red-500');
  });

  it('displays the correct battery level', () => {
    render(<Header isConnected={true} batteryLevel={75} />);

    const batteryElement = screen.getByText(/Battery:/i);
    expect(batteryElement).toBeInTheDocument();
    expect(batteryElement).toHaveTextContent('Battery: 75%');
  });
});
