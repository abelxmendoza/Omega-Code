import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import LedModal from '../src/components/LedModal';

describe('LedModal', () => {
  it('renders LedModal component and applies settings', () => {
    const sendCommand = jest.fn();
    const onClose = jest.fn();
    const { getByText } = render(<LedModal sendCommand={sendCommand} isOpen={true} onClose={onClose} />);
    fireEvent.click(getByText('Apply'));
    expect(sendCommand).toHaveBeenCalled();
  });
});
