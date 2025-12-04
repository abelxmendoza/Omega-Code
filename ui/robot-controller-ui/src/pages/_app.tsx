/*
# File: /Omega-Code/ui/robot-controller-ui/src/pages/_app.tsx
# Summary:
The custom `_app.tsx` file for the Next.js application serves as a wrapper for all pages.
It integrates global state management using Redux and CommandContext, applies global styles, 
and implements an error boundary for better runtime error handling and user experience.
*/

import React from 'react';
import Head from 'next/head';
import { AppProps } from 'next/app'; // Import Next.js App component props type
import { Provider } from 'react-redux'; // Redux Provider for global state management
import { CommandProvider } from '../context/CommandContext'; // Provides WebSocket and command state management
import { MacroProvider } from '../context/MacroContext'; // Macro builder + automation runtime
import { CapabilityProvider } from '../context/CapabilityContext'; // System capability detection
import store from '../redux/store'; // Redux store configuration
import '../styles/globals.scss'; // Import global styles
import { ErrorBoundary } from 'react-error-boundary'; // Error boundary for runtime error handling
import InstallPrompt from '../components/InstallPrompt';
import RobotOfflineBanner from '../components/RobotOfflineBanner';

/**
 * Fallback UI for the ErrorBoundary.
 * Displays an error message and instructions for recovery in case of runtime errors.
 * @param error - The caught error object.
 */
const ErrorFallback = ({ error }: { error: Error }) => (
  <div
    role="alert"
    className="flex flex-col items-center justify-center h-screen bg-red-100 text-red-900 p-8"
  >
    <h2 className="text-xl font-bold mb-4">Oops! Something went wrong.</h2>
    <pre className="bg-red-200 p-4 rounded-md text-sm whitespace-pre-wrap">
      {error.message}
    </pre>
    <p className="mt-4 text-sm text-gray-700">
      Please refresh the page or contact support if the issue persists.
    </p>
  </div>
);

/*
# Component: MyApp
The root-level wrapper for all pages in the Next.js application. 
It integrates Redux for managing application state, CommandContext for WebSocket handling,
global styles for a unified UI, and an error boundary for catching and handling runtime errors.

Props:
- Component: The page component currently being rendered.
- pageProps: Initial props passed to the page component.
*/
const MyApp = ({ Component, pageProps }: AppProps) => {
  return (
    <>
      <Head>
        <link rel="icon" type="image/png" href="/image/README/omegatechlogopro-noBackground.png" />
        <link rel="shortcut icon" type="image/png" href="/image/README/omegatechlogopro-noBackground.png" />
        <link rel="apple-touch-icon" href="/image/README/omegatechlogopro-noBackground.png" />
      </Head>
      <Provider store={store}> {/* Wrap all components with Redux state management */}
        <CapabilityProvider> {/* System capability detection and context */}
          <CommandProvider> {/* Provide WebSocket and command logging functionality */}
            <MacroProvider> {/* Persisted macro editor + runtime */}
              <ErrorBoundary FallbackComponent={ErrorFallback}> {/* Gracefully handle runtime errors */}
                <RobotOfflineBanner /> {/* Show banner when robot backend is offline */}
                <Component {...pageProps} /> {/* Render the current page */}
                <InstallPrompt /> {/* PWA install prompt */}
                {/* SpeedInsights removed - install @vercel/speed-insights package to enable */}
              </ErrorBoundary>
            </MacroProvider>
          </CommandProvider>
        </CapabilityProvider>
      </Provider>
    </>
  );
};

export default MyApp;
