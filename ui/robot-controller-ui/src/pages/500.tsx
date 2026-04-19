import Head from 'next/head';
import Link from 'next/link';
import React from 'react';

export default function FiveHundred() {
  return (
    <main className="cyber-theme min-h-screen grid place-items-center p-6">
      <Head>
        <title>500 – Server error</title>
        <meta name="robots" content="noindex" />
      </Head>

      <div className="text-center max-w-md">
        <p className="text-sm font-medium text-red-400 uppercase tracking-widest">500</p>
        <h1 className="mt-2 text-3xl font-bold tracking-tight text-white">
          Something went wrong
        </h1>
        <p className="mt-2 text-white/50 text-sm">
          The server encountered an error. It&apos;s not you — it&apos;s us.
        </p>

        <div className="mt-6 flex items-center justify-center gap-3">
          <button
            type="button"
            onClick={() => (typeof window !== 'undefined' ? window.location.reload() : null)}
            className="inline-flex items-center rounded-md bg-purple-700 hover:bg-purple-600 px-4 py-2 text-sm font-medium text-white transition-colors focus:outline-none focus:ring-2 focus:ring-purple-500"
          >
            Reload page
          </button>
          <Link
            href="/"
            className="inline-flex items-center rounded-md border border-white/20 bg-white/5 hover:bg-white/10 px-4 py-2 text-sm font-medium text-white/80 transition-colors focus:outline-none focus:ring-2 focus:ring-white/20"
          >
            Go home
          </Link>
        </div>

        <p className="mt-8 text-xs text-white/30">
          If the problem persists, check the server logs or try again later.
        </p>
      </div>
    </main>
  );
}
