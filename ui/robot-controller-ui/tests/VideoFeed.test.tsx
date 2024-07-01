import React from 'react';
import { render } from '@testing-library/react';
import VideoFeed from '../src/components/VideoFeed';
import { act } from 'react-dom/test-utils';

describe('VideoFeed', () => {
  it('renders VideoFeed component', () => {
    const { getByAltText } = render(<VideoFeed />);
    expect(getByAltText('Video Feed')).toBeInTheDocument();
  });

  act(() => {
    // fire events that update state
  });
});
