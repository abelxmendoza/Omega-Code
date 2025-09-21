// tests/VideoFeed.test.tsx
import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import VideoFeed from '@/components/VideoFeed'; // use your "@/"" alias

describe('VideoFeed', () => {
  it('renders the live video image', () => {
    render(<VideoFeed />);
    // Component renders: alt="Live video feed"
    expect(screen.getByAltText(/live video feed/i)).toBeInTheDocument();
  });
});
