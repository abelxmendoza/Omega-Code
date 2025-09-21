// tests/mswServer.ts
// Creates a real MSW server if 'msw/node' is available; otherwise a no-op server so tests don't crash.
type ServerLike = {
  listen: (opts?: any) => void;
  use: (...h: any[]) => void;
  resetHandlers: () => void;
  close: () => void;
};

let server: ServerLike = {
  listen: () => {},
  use: () => {},
  resetHandlers: () => {},
  close: () => {},
};

try {
  // Use require so Jest won't fail module resolution at import time
  // (Some setups trip over ESM/exports; this keeps tests running.)
  // eslint-disable-next-line @typescript-eslint/no-var-requires
  const { setupServer } = require('msw/node');
  server = setupServer();
} catch (_e) {
  // MSW not available or resolver mismatch — keep the no-op server.
  // You can `npm i -D msw@^2` to enable real handlers.
}

export { server };
