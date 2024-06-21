import React from 'react';
import { AppProps } from 'next/app';
import { CommandLogProvider } from '../components/CommandLogContext';
import '../styles/globals.css'; // Import your global CSS file

const MyApp = ({ Component, pageProps }: AppProps) => {
  return (
    <CommandLogProvider>
      <Component {...pageProps} />
    </CommandLogProvider>
  );
};

export default MyApp;
