import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import ColorWheel from '../src/components/ColorWheel';

describe('ColorWheel', () => {
  it('renders ColorWheel component and selects a color', () => {
    const onSelectColor = jest.fn();
    const { getByText } = render(<ColorWheel onSelectColor={onSelectColor} />);
    fireEvent.click(getByText('Select Color'));
    expect(onSelectColor).toHaveBeenCalledWith("#aabbcc");
  });
});
