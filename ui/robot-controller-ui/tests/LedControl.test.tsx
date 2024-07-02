import React from 'react';
import { render, fireEvent, act } from '@testing-library/react';
import LedControl from '../src/components/LedControl';

describe('LedControl', () => {
  it('renders LedControl component and applies settings', async () => {
    const sendCommand = jest.fn();
    let getByLabelText, getByText;

    await act(async () => {
      ({ getByLabelText, getByText } = render(<LedControl sendCommand={sendCommand} />));
    });

    // Simulate user interactions
    fireEvent.change(getByLabelText('Mode:'), { target: { value: 'multi' } });
    fireEvent.change(getByLabelText('Pattern:'), { target: { value: 'blink' } });
    fireEvent.change(getByLabelText('Interval (ms):'), { target: { value: '500' } });

    // Apply settings
    await act(async () => {
      fireEvent.click(getByText('Apply'));
    });

    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({
      command: 'set-led',
      color: '#ffffff', // Default color since we are not changing it in the test
      mode: 'multi',
      pattern: 'blink',
      interval: 500,
    }));
  });
});
