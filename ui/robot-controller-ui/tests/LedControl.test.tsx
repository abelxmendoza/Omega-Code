import React from 'react';
import { render, fireEvent, waitFor } from '@testing-library/react';
import { act } from 'react'; // Updated import
import LedControl from '../src/components/LedControl';

describe('LedControl', () => {
  it('renders LedControl component and applies settings', async () => {
    const sendCommand = jest.fn();
    let getByText;

    await act(async () => {
      const result = render(<LedControl sendCommand={sendCommand} />);
      getByText = result.getByText;
    });

    await act(async () => {
      fireEvent.click(getByText('Apply'));
    });

    await waitFor(() => {
      expect(sendCommand).toHaveBeenCalled();
    });
  });
});
