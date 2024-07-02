import React from 'react';
import { render, fireEvent, act } from '@testing-library/react';
import LedModal from '../src/components/LedModal';

describe('LedModal', () => {
  it('renders LedModal component and applies settings', async () => {
    const sendCommand = jest.fn();
    const onClose = jest.fn();
    let getByText, getByLabelText, container;

    await act(async () => {
      ({ getByText, getByLabelText, container } = render(<LedModal sendCommand={sendCommand} isOpen={true} onClose={onClose} />));
    });

    // Change color using SketchPicker
    const colorPicker = container.querySelector('.sketch-picker');
    if (colorPicker) {
      const colorInput = colorPicker.querySelector('input[type="text"]');
      if (colorInput) {
        await act(async () => {
          fireEvent.change(colorInput, { target: { value: '#ff0000' } });
          fireEvent.blur(colorInput); // Trigger blur to simulate color change
        });
      }
    }

    // Change mode
    await act(async () => {
      fireEvent.change(getByLabelText('Mode:'), { target: { value: 'multi' } });
    });

    // Change pattern
    await act(async () => {
      fireEvent.change(getByLabelText('Pattern:'), { target: { value: 'blink' } });
    });

    // Change interval
    await act(async () => {
      fireEvent.change(getByLabelText('Interval (ms):'), { target: { value: '500' } });
    });

    // Apply settings
    await act(async () => {
      fireEvent.click(getByText('Apply'));
    });

    expect(sendCommand).toHaveBeenCalledWith(JSON.stringify({
      command: 'set-led',
      color: '#ff0000',
      mode: 'multi',
      pattern: 'blink',
      interval: 500,
    }));
  });

  it('closes the modal when the close button is clicked', async () => {
    const sendCommand = jest.fn();
    const onClose = jest.fn();
    let getByText;

    await act(async () => {
      ({ getByText } = render(<LedModal sendCommand={sendCommand} isOpen={true} onClose={onClose} />));
    });

    await act(async () => {
      fireEvent.click(getByText('X'));
    });

    expect(onClose).toHaveBeenCalled();
  });
});
