import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import LightingMode from '../src/components/LightingMode';

describe('LightingMode', () => {
  it('renders LightingMode component and selects a mode', () => {
    const onSelectMode = jest.fn();
    const { getByText } = render(<LightingMode onSelectMode={onSelectMode} />);
    fireEvent.click(getByText('Single Color'));
    expect(onSelectMode).toHaveBeenCalledWith('single');
  });
});
