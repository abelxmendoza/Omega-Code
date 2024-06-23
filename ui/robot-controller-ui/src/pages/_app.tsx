// File: /Omega-Code/ui/robot-controller-ui/src/pages/_app.tsx

/*
This is the custom App component for the Next.js application.
It wraps all pages with the CommandLogProvider to manage the command log state across the app.
It also includes global CSS styles for consistent styling throughout the application.
*/

// File: /src/pages/_app.tsx

// File: /Omega-Code/ui/robot-controller-ui/src/pages/_app.tsx
import React from 'react';
import { AppProps } from 'next/app';
import { Provider } from 'react-redux';
import { CommandLogProvider } from '../components/CommandLogContext';
import store from '../redux/store';
import '../styles/globals.css';

const MyApp = ({ Component, pageProps }: AppProps) => {
  return (
    <Provider store={store}>
      <CommandLogProvider>
        <Component {...pageProps} />
      </CommandLogProvider>
    </Provider>
  );
};

export default MyApp;
