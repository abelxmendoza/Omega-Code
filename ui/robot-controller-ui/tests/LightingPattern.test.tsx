import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import LightingPattern from '../src/components/LightingPattern';

describe('LightingPattern', () => {
  it('renders LightingPattern component and selects a pattern', () => {
    const onSelectPattern = jest.fn();
    const { getByText } = render(<LightingPattern onSelectPattern={onSelectPattern} />);
    fireEvent.click(getByText('Static'));
    expect(onSelectPattern).toHaveBeenCalledWith('static');
  });
});
