import React from 'react';
import { render, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import GpsLocation from '../src/components/GpsLocation';

jest.mock('../src/components/MapComponent', () => () => <div>Mocked MapComponent</div>);

describe('GpsLocation', () => {
  it('renders GpsLocation component with MapComponent', async () => {
    const { getByText } = render(<GpsLocation />);

    // Check if the GPS Location title is rendered
    expect(getByText('GPS Location')).toBeInTheDocument();

    // Wait for the MapComponent to be rendered
    await waitFor(() => {
      expect(getByText('Mocked MapComponent')).toBeInTheDocument();
    });
  });
});