import { Server, WebSocket as MockWS } from 'mock-socket';

export function setupWsHarness(url = 'ws://localhost:8765') {
  let lastSocket: any;
  let server: Server;

  beforeAll(() => {
    (globalThis as any).WebSocket = MockWS as any;
    process.env.NEXT_PUBLIC_MOVEMENT_WS_URL = url;

    server = new Server(url);
    server.on('connection', (sock) => {
      lastSocket = sock;
      sock.send(JSON.stringify({ type: 'hello', v: 1 }));
    });
  });

  afterAll(() => server?.close());

  return {
    getSocket: () => lastSocket,
    send: (msg: any) => lastSocket?.send(JSON.stringify(msg)),
    server: () => server,
  };
}
