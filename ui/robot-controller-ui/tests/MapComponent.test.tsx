import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import { renderWithProviders } from './utils/test-helpers';
import MapComponent from '../src/components/MapComponent';

describe('MapComponent', () => {
  test('renders map component placeholder', () => {
    const { container } = renderWithProviders(<MapComponent />);
    // Component renders a placeholder div, not #map
    expect(container.querySelector('.bg-gray-800')).toBeInTheDocument();
    expect(screen.getByText(/Map Component/i)).toBeInTheDocument();
  });

  test('renders with demo mode', () => {
    const { container } = renderWithProviders(<MapComponent dummy={true} />);
    expect(screen.getByText(/Demo Mode/i)).toBeInTheDocument();
  });

  test('renders with GPS disabled message', () => {
    const { container } = renderWithProviders(<MapComponent />);
    expect(screen.getByText(/GPS Disabled/i)).toBeInTheDocument();
  });
});
