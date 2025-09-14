// tests/setupTests.ts
// Extends RTL matchers and provides stable browser-ish APIs for JSDOM tests.
import '@testing-library/jest-dom'
import 'whatwg-fetch' // polyfills fetch/Request/Response in JSDOM

// --- OPTIONAL toggles (uncomment when needed) --------------------------------
// MSW for API mocking across tests (recommended for /api/*):
// import { setupServer } from 'msw/node'
// export const server = setupServer()
// beforeAll(() => server.listen({ onUnhandledRequest: 'bypass' }))
// afterEach(() => server.resetHandlers())
// afterAll(() => server.close())

// If some components rely on createObjectURL (e.g., camera blobs), provide stubs:
if (typeof URL !== 'undefined') {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const U: any = URL
  if (!U.createObjectURL) U.createObjectURL = () => 'blob://test'
  if (!U.revokeObjectURL) U.revokeObjectURL = () => {}
}

// If a lib expects ResizeObserver in JSDOM:
if (typeof (globalThis as any).ResizeObserver === 'undefined') {
  ;(globalThis as any).ResizeObserver = class {
    observe() {}
    unobserve() {}
    disconnect() {}
  }
}

// Keep the module a TS file
export {}
