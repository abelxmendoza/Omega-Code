import React from 'react';
import Head from 'next/head';
import { AppProps } from 'next/app';
import { CommandProvider } from '../context/CommandContext';
import { MacroProvider } from '../context/MacroContext';
import { CapabilityProvider } from '../context/CapabilityContext';
import { SystemHealthProvider } from '../context/SystemHealthContext';
import { SystemModeProvider } from '../context/SystemModeContext';
import { DemoModeProvider } from '../context/DemoModeContext';
import '../styles/globals.scss';
import { ErrorBoundary, FallbackProps } from 'react-error-boundary';
import InstallPrompt from '../components/InstallPrompt';
import RobotOfflineBanner from '../components/RobotOfflineBanner';

/**
 * Fallback UI for the ErrorBoundary.
 * Displays an error message and instructions for recovery in case of runtime errors.
 * @param error - The caught error object.
 */
const ErrorFallback = ({ error }: FallbackProps) => (
  <div
    role="alert"
    className="flex flex-col items-center justify-center h-screen bg-red-100 text-red-900 p-8"
  >
    <h2 className="text-xl font-bold mb-4">Oops! Something went wrong.</h2>
    <pre className="bg-red-200 p-4 rounded-md text-sm whitespace-pre-wrap">
      {error instanceof Error ? error.message : String(error)}
    </pre>
    <p className="mt-4 text-sm text-gray-700">
      Please refresh the page or contact support if the issue persists.
    </p>
  </div>
);

const MyApp = ({ Component, pageProps }: AppProps) => {
  return (
    <>
      <Head>
        <link rel="icon" type="image/png" href="/image/README/omegatechlogopro-noBackground.png" />
        <link rel="shortcut icon" type="image/png" href="/image/README/omegatechlogopro-noBackground.png" />
        <link rel="apple-touch-icon" href="/image/README/omegatechlogopro-noBackground.png" />
      </Head>
      <DemoModeProvider>
      <SystemHealthProvider>
        <SystemModeProvider>
        <CapabilityProvider>
          <CommandProvider>
            <MacroProvider>
              <ErrorBoundary FallbackComponent={ErrorFallback}>
                <RobotOfflineBanner />
                <Component {...pageProps} />
                <InstallPrompt />
              </ErrorBoundary>
            </MacroProvider>
          </CommandProvider>
        </CapabilityProvider>
        </SystemModeProvider>
      </SystemHealthProvider>
      </DemoModeProvider>
    </>
  );
};

export default MyApp;
