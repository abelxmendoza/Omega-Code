import React from 'react';
import { render } from '@testing-library/react';
import VideoFeed from '../src/components/VideoFeed';

describe('VideoFeed', () => {
  it('renders VideoFeed component', () => {
    const { getByAltText } = render(<VideoFeed />);
    expect(getByAltText('Video Feed')).toBeInTheDocument();
  });
});

