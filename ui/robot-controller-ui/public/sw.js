// Service Worker for Omega Robot Controller PWA
const CACHE_NAME = 'omega-robot-v1';
const urlsToCache = [
  '/',
  '/manifest.json',
  '/image/README/omegatechlogopro-noBackground.png',
];

// Install event - cache resources
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(urlsToCache))
      .then(() => self.skipWaiting())
  );
});

// Activate event - clean up old caches
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((cacheNames) => {
      return Promise.all(
        cacheNames
          .filter((name) => name !== CACHE_NAME)
          .map((name) => caches.delete(name))
      );
    }).then(() => self.clients.claim())
  );
});

// Fetch event - serve from cache, fallback to network
self.addEventListener('fetch', (event) => {
  // Skip non-GET requests and WebSocket connections
  if (event.request.method !== 'GET' || event.request.url.startsWith('ws://') || event.request.url.startsWith('wss://')) {
    return;
  }

  // Skip Next.js dev server assets and HMR
  if (event.request.url.includes('/_next/') || event.request.url.includes('/__webpack') || event.request.url.includes('webpack-hmr') || event.request.url.includes('hot-update')) {
    return;
  }

  // Skip localhost in development (let Next.js handle it)
  if (event.request.url.includes('localhost') || event.request.url.includes('127.0.0.1')) {
ควร return;
  }

  event.respondWith(
    caches.match(event.request)
      .then((response) => {
        // Return cached version or fetch from network
        return response || fetch(event.request).then((fetchResponse) => {
          // Don't cache non-successful responses
          if (!fetchResponse || fetchResponse.status !== 200 || fetchResponse.type !== 'basic') {
            return fetchResponse;
          }

          // Clone the response
          const responseToCache = fetchResponse.clone();

          caches.open(CACHE_NAME).then((cache) => {
            cache.put(event.request, responseToCache);
          });

          return fetchResponse;
        });
      })
      .catch(() => {
        // Return offline page if available
        return caches.match('/');
      })
  );
});

