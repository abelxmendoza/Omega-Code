/*
# File: /src/pages/500.tsx
# Summary:
#   Custom 500 page for the Pages Router.
#   - Static (no data fetching) so it can be pre-rendered
#   - Shown only in production for server-side errors
#   - Offers Reload / Home actions
*/

import Head from 'next/head';
import Link from 'next/link';
import React from 'react';

export default function FiveHundred() {
  return (
    <main className="min-h-screen grid place-items-center bg-gray-50 text-gray-800 p-6">
      <Head>
        <title>500 – Server error</title>
        <meta name="robots" content="noindex" />
      </Head>

      <div className="text-center max-w-md">
        <p className="text-sm font-medium text-gray-500">500</p>
        <h1 className="mt-2 text-3xl font-semibold tracking-tight">
          Something went wrong
        </h1>
        <p className="mt-2 text-gray-600">
          The server encountered an error. It’s not you—it&apos;s us.
        </p>

        <div className="mt-6 flex items-center justify-center gap-3">
          <button
            type="button"
            onClick={() => (typeof window !== 'undefined' ? window.location.reload() : null)}
            className="inline-flex items-center rounded-md bg-gray-900 px-4 py-2 text-sm font-medium text-white hover:bg-gray-800 focus:outline-none focus:ring-2 focus:ring-gray-400"
          >
            Reload page
          </button>
          <Link
            href="/"
            className="inline-flex items-center rounded-md border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-800 hover:bg-gray-100 focus:outline-none focus:ring-2 focus:ring-gray-300"
          >
            Go home
          </Link>
        </div>

        <p className="mt-8 text-xs text-gray-500">
          If the problem persists, check the server logs or try again later.
        </p>
      </div>
    </main>
  );
}
