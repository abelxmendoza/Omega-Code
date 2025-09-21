// tests/setupPolyfills.ts
// Runs *before* any other test code (via jest.config.js -> setupFiles)

import { TextEncoder, TextDecoder } from 'util';
if (!('TextEncoder' in globalThis)) (globalThis as any).TextEncoder = TextEncoder;
if (!('TextDecoder' in globalThis)) (globalThis as any).TextDecoder = TextDecoder as unknown as typeof globalThis.TextDecoder;

// Optional: ensure Web Crypto exists (some libs expect it)
try {
  if (!('crypto' in globalThis)) {
    // eslint-disable-next-line @typescript-eslint/no-var-requires
    (globalThis as any).crypto = require('crypto').webcrypto;
  }
} catch { /* ignore */ }

// ✅ NEW: Minimal BroadcastChannel polyfill for MSW v2 in Node/JSDOM
if (!('BroadcastChannel' in globalThis)) {
  class FakeBroadcastChannel {
    name: string;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    onmessage: any = null;
    constructor(name: string) { this.name = name; }
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    postMessage(_msg: any) { /* no-op */ }
    close() { /* no-op */ }
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    addEventListener(_type: string, _handler: any) { /* no-op */ }
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    removeEventListener(_type: string, _handler: any) { /* no-op */ }
  }
  (globalThis as any).BroadcastChannel = FakeBroadcastChannel as any;
}
