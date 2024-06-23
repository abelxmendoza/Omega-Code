import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import LedControl from '../src/components/LedControl';

describe('LedControl', () => {
  it('renders LedControl component and applies settings', () => {
    const sendCommand = jest.fn();
    const { getByText } = render(<LedControl sendCommand={sendCommand} />);
    fireEvent.click(getByText('Apply'));
    expect(sendCommand).toHaveBeenCalled();
  });
});
