import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import SpeedControl from '../src/components/SpeedControl';

describe('SpeedControl', () => {
  it('opens LED modal on single click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByText } = render(
      <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
    );

    const ledButton = getByText((content, element) => 
      element.tagName.toLowerCase() === 'button' && content.includes('I') && content.includes('(LED)')
    );
    fireEvent.click(ledButton);
    expect(mockOnOpenLedModal).toHaveBeenCalled();
  });

  it('toggles LED on double click', () => {
    const mockSendCommand = jest.fn();
    const mockOnOpenLedModal = jest.fn();

    const { getByText } = render(
      <SpeedControl sendCommand={mockSendCommand} onOpenLedModal={mockOnOpenLedModal} />
    );

    const ledButton = getByText((content, element) => 
      element.tagName.toLowerCase() === 'button' && content.includes('I') && content.includes('(LED)')
    );

    fireEvent.doubleClick(ledButton);
    expect(mockSendCommand).toHaveBeenCalledWith('toggle-led');
  });
});
