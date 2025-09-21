import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import IntervalTiming from '../src/components/IntervalTiming';

describe('IntervalTiming', () => {
  it('renders IntervalTiming component and sets interval', () => {
    const onSetInterval = jest.fn();
    const { getByLabelText } = render(<IntervalTiming onSetInterval={onSetInterval} />);
    fireEvent.change(getByLabelText('Interval (ms):'), { target: { value: '2000' } });
    expect(onSetInterval).toHaveBeenCalledWith(2000);
  });
});
