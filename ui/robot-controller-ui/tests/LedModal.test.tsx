import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import { act } from 'react'; // Update this import
import LedModal from '../src/components/LedModal';

describe('LedModal', () => {
  it('renders LedModal component and applies settings', () => {
    const sendCommand = jest.fn();
    const onClose = jest.fn();
    let getByText;
    act(() => {
      const result = render(<LedModal sendCommand={sendCommand} isOpen={true} onClose={onClose} />);
      getByText = result.getByText;
    });
    act(() => {
      fireEvent.click(getByText('Apply'));
    });
    expect(sendCommand).toHaveBeenCalled();
  });
});
