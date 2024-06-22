import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import SpeedControl from '../src/components/SpeedControl';

describe('SpeedControl', () => {
  it('opens LED modal on single click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByRole } = render(
      <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
    );

    const ledButton = getByRole('button', { name: /I \(LED\)/i });
    fireEvent.click(ledButton);
    expect(mockOnOpenLedModal).toHaveBeenCalled();
  });

  it('toggles LED on double click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByRole } = render(
      <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
    );

    const ledButton = getByRole('button', { name: /I \(LED\)/i });

    fireEvent.doubleClick(ledButton);
    expect(mockSendCommand).toHaveBeenCalledWith('toggle-led');
  });
});
