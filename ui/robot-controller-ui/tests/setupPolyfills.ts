// tests/setupPolyfills.ts
// Runs *before* any other test code (via jest.config.js -> setupFiles)

import { TextEncoder, TextDecoder } from 'util';

// Use Object.defineProperty to avoid TypeScript assertion syntax issues with Babel
if (!('TextEncoder' in globalThis)) {
  Object.defineProperty(globalThis, 'TextEncoder', {
    value: TextEncoder,
    writable: true,
    configurable: true,
  });
}
if (!('TextDecoder' in globalThis)) {
  Object.defineProperty(globalThis, 'TextDecoder', {
    value: TextDecoder,
    writable: true,
    configurable: true,
  });
}

// Optional: ensure Web Crypto exists (some libs expect it)
try {
  if (!('crypto' in globalThis)) {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    const crypto = require('crypto');
    Object.defineProperty(globalThis, 'crypto', {
      value: crypto.webcrypto,
      writable: true,
      configurable: true,
    });
  }
} catch {
  // ignore
}

// ✅ NEW: Minimal BroadcastChannel polyfill for MSW v2 in Node/JSDOM
if (!('BroadcastChannel' in globalThis)) {
  class FakeBroadcastChannel {
    name: string;
    onmessage: any = null;
    constructor(name: string) {
      this.name = name;
    }
    postMessage(_msg: any) {
      /* no-op */
    }
    close() {
      /* no-op */
    }
    addEventListener(_type: string, _handler: any) {
      /* no-op */
    }
    removeEventListener(_type: string, _handler: any) {
      /* no-op */
    }
  }
  Object.defineProperty(globalThis, 'BroadcastChannel', {
    value: FakeBroadcastChannel,
    writable: true,
    configurable: true,
  });
}
