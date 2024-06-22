// File: /Omega-Code/ui/robot-controller-ui/src/pages/_app.tsx

/*
This is the custom App component for the Next.js application.
It wraps all pages with the CommandLogProvider to manage the command log state across the app.
It also includes global CSS styles for consistent styling throughout the application.
*/

import React from 'react';
import { AppProps } from 'next/app';
import { CommandLogProvider } from '../components/CommandLogContext';
import '../styles/globals.css'; // Import your global CSS file

const MyApp = ({ Component, pageProps }: AppProps) => {
  return (
    // Wrap all pages with CommandLogProvider to manage command log state globally
    <CommandLogProvider>
      {/* Render the current page */}
      <Component {...pageProps} />
    </CommandLogProvider>
  );
};

export default MyApp;
