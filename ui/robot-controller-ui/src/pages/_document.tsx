import React from 'react';
import { Html, Head, Main, NextScript } from 'next/document';

export default function Document() {
  return (
    <Html lang="en">
      <Head>
        <link rel="icon" href="/image/README/omegatechlogopro-noBackground.png" type="image/png" />
        <link rel="apple-touch-icon" href="/image/README/omegatechlogopro-noBackground.png" />
        <link rel="manifest" href="/manifest.json" />
        <meta name="theme-color" content="#C400FF" />
        <meta name="apple-mobile-web-app-capable" content="yes" />
        <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent" />
        <meta name="apple-mobile-web-app-title" content="Omega Robot" />
        <meta name="mobile-web-app-capable" content="yes" />
        <meta name="application-name" content="Omega Robot" />
      </Head>
      <body>
        <Main />
        <NextScript />
        <script
          dangerouslySetInnerHTML={{
            __html: `
              if (typeof window !== 'undefined' && 'serviceWorker' in navigator) {
                var isProduction = window.location.hostname !== 'localhost' && window.location.hostname !== '127.0.0.1';
                if (isProduction) {
                window.addEventListener('load', function() {
                  navigator.serviceWorker.register('/sw.js')
                    .then(function(registration) {
                      console.log('ServiceWorker registration successful');
                    }, function(err) {
                      console.log('ServiceWorker registration failed: ', err);
                    });
                });
                } else {
                  // Unregister service worker in development
                  navigator.serviceWorker.getRegistrations().then(function(registrations) {
                    for(var i = 0; i < registrations.length; i++) {
                      registrations[i].unregister();
                    }
                  });
                }
              }
            `,
          }}
        />
      </body>
    </Html>
  );
}

