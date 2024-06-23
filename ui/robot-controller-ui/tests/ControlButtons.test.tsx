import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import ControlButtons from '../src/components/ControlButtons';

describe('ControlButtons', () => {
  it('renders ControlButtons component and triggers callbacks', () => {
    const onStart = jest.fn();
    const onStop = jest.fn();
    const onApply = jest.fn();
    const { getByText } = render(<ControlButtons onStart={onStart} onStop={onStop} onApply={onApply} />);
    fireEvent.click(getByText('Start'));
    expect(onStart).toHaveBeenCalled();
    fireEvent.click(getByText('Stop'));
    expect(onStop).toHaveBeenCalled();
    fireEvent.click(getByText('Apply Settings'));
    expect(onApply).toHaveBeenCalled();
  });
});
