/*
# File: /src/pages/404.tsx
# Summary:
#   Lightweight custom 404 page for the Pages Router.
#   - Accessible, responsive, Tailwind-styled
#   - Works with dark or light backgrounds
#   - Provides "Go home" and "Go back" actions
*/

import Head from 'next/head';
import Link from 'next/link';
import React from 'react';

export default function FourOhFour() {
  return (
    <main className="min-h-screen grid place-items-center bg-gray-50 text-gray-800 p-6">
      <Head>
        <title>404 – Page not found</title>
        <meta name="robots" content="noindex" />
      </Head>

      <div className="text-center">
        <p className="text-sm font-medium text-gray-500">404</p>
        <h1 className="mt-2 text-3xl font-semibold tracking-tight">
          Page not found
        </h1>
        <p className="mt-2 text-gray-600">
          The page you’re looking for doesn’t exist or was moved.
        </p>

        <div className="mt-6 flex items-center justify-center gap-3">
          <Link
            href="/"
            className="inline-flex items-center rounded-md bg-gray-900 px-4 py-2 text-sm font-medium text-white hover:bg-gray-800 focus:outline-none focus:ring-2 focus:ring-gray-400"
          >
            Go home
          </Link>
          <button
            type="button"
            onClick={() => (typeof window !== 'undefined' ? window.history.back() : null)}
            className="inline-flex items-center rounded-md border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-800 hover:bg-gray-100 focus:outline-none focus:ring-2 focus:ring-gray-300"
          >
            Go back
          </button>
        </div>

        <div className="mt-8 text-xs text-gray-500">
          <code>
            {typeof window !== 'undefined' ? window.location.pathname : '/'}
          </code>
        </div>
      </div>
    </main>
  );
}
