import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import Status from '../src/components/Status';

describe('Status', () => {
  it('displays the correct status icon and text when connected', () => {
    render(<Status status="Connected" battery={75} />);

    const statusElement = screen.getByText(/Status:/i).parentElement;
    expect(statusElement).toBeInTheDocument();
    expect(screen.getByText(/Status:/i)).toBeInTheDocument();
    expect(screen.getByText(/Battery:/i)).toBeInTheDocument();
    expect(screen.getByText(/75%/i)).toBeInTheDocument();
    expect(screen.getByTestId('status-icon')).toHaveClass('text-green-500');
  });

  it('displays the correct status icon and text when disconnected', () => {
    render(<Status status="Disconnected" battery={15} />);

    const statusElement = screen.getByText(/Status:/i).parentElement;
    expect(statusElement).toBeInTheDocument();
    expect(screen.getByText(/Status:/i)).toBeInTheDocument();
    expect(screen.getByText(/Battery:/i)).toBeInTheDocument();
    expect(screen.getByText(/15%/i)).toBeInTheDocument();
    expect(screen.getByTestId('status-icon')).toHaveClass('text-red-500');
  });

  it('displays the correct battery bar color based on battery level', () => {
    const { rerender } = render(<Status status="Connected" battery={75} />);
    const batteryBar = screen.getByText(/75%/i).previousElementSibling.firstChild;
    expect(batteryBar).toHaveClass('bg-blue-500');

    rerender(<Status status="Connected" battery={15} />);
    const batteryBarUpdated = screen.getByText(/15%/i).previousElementSibling.firstChild;
    expect(batteryBarUpdated).toHaveClass('bg-red-500');
  });
});
