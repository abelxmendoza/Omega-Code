import React from 'react';
import { render } from '@testing-library/react';
import Header from '../src/components/Header';

describe('Header', () => {
  it('renders Header component with connection status and battery level', () => {
    const { getByText } = render(<Header isConnected={true} batteryLevel={50} />);
    expect(getByText('Robot Controller')).toBeInTheDocument();
    expect(getByText('Status:')).toBeInTheDocument();
    expect(getByText('Battery:')).toBeInTheDocument();
    expect(getByText('50%')).toBeInTheDocument();
  });
});
