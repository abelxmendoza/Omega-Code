import React from 'react';
import { render } from '@testing-library/react';
import Status from '../src/components/Status';

describe('Status', () => {
  it('renders Status component with status and battery level', () => {
    const { getByText } = render(<Status status="Connected" battery={50} />);
    expect(getByText('Status:')).toBeInTheDocument();
    expect(getByText('Battery:')).toBeInTheDocument();
    expect(getByText('50%')).toBeInTheDocument();
  });
});
