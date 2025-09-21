// tests/setupTests.ts
import '@testing-library/jest-dom'
import 'whatwg-fetch'

// --- MSW: global server + default handlers ---
import { server } from './mswServer'
import { handlers } from './handlers/default'

beforeAll(() => server.listen({ onUnhandledRequest: 'bypass' }))
beforeEach(() => server.use(...handlers)) // reset defaults each test
afterEach(() => server.resetHandlers())
afterAll(() => server.close())

// WebSocket Mock
const mockWebSocket = {
  addEventListener: jest.fn(),
  removeEventListener: jest.fn(),
  send: jest.fn(),
  close: jest.fn(),
  readyState: WebSocket.CONNECTING,
  CONNECTING: WebSocket.CONNECTING,
  OPEN: WebSocket.OPEN,
  CLOSING: WebSocket.CLOSING,
  CLOSED: WebSocket.CLOSED,
};

const WebSocketMock = jest.fn(() => mockWebSocket);
WebSocketMock.CONNECTING = WebSocket.CONNECTING;
WebSocketMock.OPEN = WebSocket.OPEN;
WebSocketMock.CLOSING = WebSocket.CLOSING;
WebSocketMock.CLOSED = WebSocket.CLOSED;

(global as any).WebSocket = WebSocketMock;
(global as any).WebSocket.mock = {
  instances: [mockWebSocket]
};

// If some components rely on createObjectURL (e.g., camera blobs), provide stubs:
if (typeof URL !== 'undefined') {
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

// (Optional) quiet next/dynamic act() warnings in tests
// jest.mock('next/dynamic', () => (importer: any) => {
//   try {
//     const mod = importer()
//     if (mod && typeof (mod as any).then === 'function') return () => null
//     return (mod as any).default ?? mod
//   } catch {
//     return () => null
//   }
// })

export { server } // so tests can override handlers
