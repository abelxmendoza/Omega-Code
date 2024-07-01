import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import SpeedControl from '../src/components/SpeedControl';
import { CommandLogProvider } from '../src/components/CommandLogContext';

describe('SpeedControl', () => {
  it('opens LED modal on single click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByTestId } = render(
      <CommandLogProvider>
        <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
      </CommandLogProvider>
    );

    const ledButton = getByTestId('led-button');
    fireEvent.click(ledButton);
    expect(mockOnOpenLedModal).toHaveBeenCalled();
  });

  it('toggles LED on double click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByTestId } = render(
      <CommandLogProvider>
        <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
      </CommandLogProvider>
    );

    const ledButton = getByTestId('led-button');
    fireEvent.doubleClick(ledButton);
    expect(mockSendCommand).toHaveBeenCalledWith('toggle-led');
  });
});


