import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
// Next.js pages
import NotFoundPage from '@/pages/404';
import ErrorPage from '@/pages/500';

describe('Error pages', () => {
  it('404 renders a friendly not-found message', () => {
    render(<NotFoundPage />);
    const heading = screen.getByRole('heading');
    expect(heading).toBeInTheDocument();
    // Most pages include a way home
    const maybeHome = screen.queryByRole('link', { name: /home|back/i });
    if (maybeHome) expect(maybeHome).toBeInTheDocument();
  });

  it('500 renders a friendly failure message', () => {
    render(<ErrorPage />);
    const heading = screen.getByRole('heading');
    expect(heading).toBeInTheDocument();
  });
});
