import Head from 'next/head';
import Link from 'next/link';
import React from 'react';

export default function FourOhFour() {
  return (
    <main className="cyber-theme min-h-screen grid place-items-center p-6">
      <Head>
        <title>404 – Page not found</title>
        <meta name="robots" content="noindex" />
      </Head>

      <div className="text-center">
        <p className="text-sm font-medium text-purple-400 uppercase tracking-widest">404</p>
        <h1 className="mt-2 text-3xl font-bold tracking-tight text-white">
          Page not found
        </h1>
        <p className="mt-2 text-white/50 text-sm">
          The page you&apos;re looking for doesn&apos;t exist or was moved.
        </p>

        <div className="mt-6 flex items-center justify-center gap-3">
          <Link
            href="/"
            className="inline-flex items-center rounded-md bg-purple-700 hover:bg-purple-600 px-4 py-2 text-sm font-medium text-white transition-colors focus:outline-none focus:ring-2 focus:ring-purple-500"
          >
            Go home
          </Link>
          <button
            type="button"
            onClick={() => (typeof window !== 'undefined' ? window.history.back() : null)}
            className="inline-flex items-center rounded-md border border-white/20 bg-white/5 hover:bg-white/10 px-4 py-2 text-sm font-medium text-white/80 transition-colors focus:outline-none focus:ring-2 focus:ring-white/20"
          >
            Go back
          </button>
        </div>

        <div className="mt-8 text-xs text-white/30 font-mono">
          {typeof window !== 'undefined' ? window.location.pathname : '/'}
        </div>
      </div>
    </main>
  );
}
