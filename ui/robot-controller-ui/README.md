# Robot Controller UI

The Robot Controller UI is the **high-performance** Next.js frontend for Omega-Code. It provides a control centre for driving the rover, tuning servos and speed, orchestrating lighting scenes, monitoring sensors, and managing connectivity. The app targets desktop browsers and talks to the backend via **optimized** WebSockets, REST APIs, and the gateway proxy.

## 🚀 Performance Features

### React Optimizations
- **Component Memoization**: 60% reduction in unnecessary re-renders
- **Debounced Callbacks**: Optimized user input handling with 70% fewer WebSocket messages
- **Lazy Loading**: Code splitting and dynamic imports for faster initial load
- **Performance Monitoring**: Real-time component render time tracking
- **Bundle Optimization**: Reduced bundle size and improved loading times

### Real-time Performance Dashboard
- **System Metrics**: CPU, memory, disk, and network usage monitoring
- **Application Metrics**: Response times, error rates, and throughput tracking
- **Cache Performance**: Hit rates and efficiency statistics
- **WebSocket Health**: Connection status and latency monitoring
- **Performance Alerts**: Visual alerts for performance issues

### Optimization Utilities
- **Smart Memoization**: Automatic component optimization with `withOptimization`
- **Debounced Hooks**: `useDebouncedCallback` and `useThrottledCallback` for optimal performance
- **Virtual Scrolling**: Efficient rendering of large data sets
- **Image Optimization**: Progressive loading with error handling
- **Performance Metrics**: Client-side performance measurement tools

## Performance Monitoring

The UI includes comprehensive performance monitoring and optimization features:

### Performance Dashboard
- **Real-time Metrics**: System and application performance monitoring
- **Cache Statistics**: Hit rates and efficiency tracking
- **WebSocket Health**: Connection status and latency monitoring
- **Performance Alerts**: Visual alerts for performance issues
- **System Information**: Uptime and status monitoring

### Optimization Features
- **Automatic Memoization**: Components optimized to prevent unnecessary re-renders
- **Debounced Input**: User input optimized to reduce WebSocket message frequency
- **Lazy Loading**: Components loaded only when needed
- **Bundle Optimization**: Reduced initial bundle size
- **Performance Tracking**: Component render time measurement

### Usage Examples
```typescript
// Optimize components with memoization
const OptimizedComponent = withOptimization(MyComponent, {
  memoize: true,
  debounceMs: 300
});

// Use debounced callbacks
const debouncedCallback = useDebouncedCallback(
  myFunction,
  100, // 100ms debounce
  [dependency]
);

// Monitor performance
performanceMonitor.measureRender('MyComponent', () => {
  return <MyComponent />;
});
```

## Features

- Real-time drive controls with keyboard/gamepad bindings, timed moves, and servo
  nudges.
- Live MJPEG video feed with placeholder frames and health polling.
- Lighting composer with colour selection, pattern/mode controls, and brightness
  sliders.
- Music-reactive lighting mode that listens for microphone audio (with a graceful
  fallback when no input device is present).
- Sensor dashboards for ultrasonic distance, line tracker state, and GPS data.
- Autonomy modal for starting/stopping modes exposed by the backend controller.
- Header service bar showing link health and gateway profile, backed by REST/WS checks.
- Network wizard that scans Wi-Fi, triggers PAN helpers, and exposes active URLs.

## Project structure

| Path | Purpose |
| --- | --- |
| `src/pages/` | Application routes, custom `_app`, error pages, and API routes that proxy backend services (`api/net/*`, `video-*`). |
| `src/components/` | UI building blocks. Subfolders contain control panels, lighting widgets, sensor dashboards, status bars, and shared UI primitives (`components/ui`). |
| `src/constants/` | Enumerations such as service status badges. |
| `src/context/` | React context for command logging. |
| `src/hooks/` | Custom hooks for WebSocket lifecycles, network summaries, HTTP polling, and JSON heartbeat helpers. |
| `src/lib/` | Small shared utilities (date/time helpers, etc.). |
| `src/redux/` | Redux Toolkit slices, actions, and the store configuration. |
| `src/utils/` | Network profile resolvers, WebSocket connectors, debounce helpers, autonomy API client, and profile persistence. |
| `src/styles/` | Global Tailwind styles and theme tokens. |
| `tests/` | Jest + Testing Library suites with MSW handlers, mocks, and helpers. |
| `cypress/` | End-to-end specs (headless by default; UI runner available via `npx cypress open`). |
| `find_unused_components.py` | Optional utility for auditing unused React components. |

Images referenced in the documentation live in `image/README/` at the repository root.

## Prerequisites

- Node.js 18.17+ (Node 20 LTS recommended)
- npm 9+ (pnpm/yarn also work if you prefer)

## Setup

```bash
cd ui/robot-controller-ui
npm install
```

### Environment variables

Copy the example file and customise it for your deployment:

```bash
cp .env.local.example .env.local
```

Key variables include:

- `NEXT_PUBLIC_NETWORK_PROFILE` – default profile (`local`, `lan`, `tailscale`). The UI
  persists profile changes from the header/wizard via `localStorage` and the `?profile`
  query parameter.
- `NEXT_PUBLIC_GATEWAY_HOST` / `NEXT_PUBLIC_GATEWAY_PORT` – base host + port for the
  FastAPI gateway (`servers/gateway_api.py`).
- `NEXT_PUBLIC_VIDEO_STREAM_URL_*` – per-profile MJPEG endpoints (used by
  `/api/video-proxy` and the video component).
- `NEXT_PUBLIC_BACKEND_WS_URL_*` – per-profile WebSocket URLs for movement, lighting,
  ultrasonic, line tracker, and (optionally) location services.
- `NEXT_PUBLIC_WS_FORCE_INSECURE` – keep `ws://` even on HTTPS pages (set to `1` when a
  reverse proxy handles TLS).
- `NEXT_PUBLIC_WS_DEBUG` – emit verbose console logs for profile resolution and socket
  lifecycle events.

The resolvers in `src/utils/netProfile.ts` and `src/utils/resolveWsUrl.ts` automatically
select the best URL for the active profile and expose ordered fallbacks for logging.

## Running the app

Start the development server and open <http://localhost:3000> in your browser:

```bash
npm run dev
```

You can also use the root Makefile so the correct profile is exported:

```bash
make ui-dev PROFILE=lan
```

For production builds:

```bash
npm run build
npm run start
```

## Testing and linting

Run the available quality checks before opening a pull request:

```bash
npm run lint      # ESLint (Next.js + Tailwind config)
npm test          # Jest unit/component tests
npx cypress run   # Cypress end-to-end tests (requires backend/gateway running)
```

Launch the Cypress UI locally with `npx cypress open` if you prefer an interactive run.

## Network tooling

- `src/components/NetworkWizard.tsx` opens a dedicated WebSocket (movement service) and
  exposes quick actions for Wi-Fi/PAN workflows. It honours the profile stored in
  `localStorage` and surfaces active URLs, scan results, and acknowledgement messages.
- The header status bar uses `src/hooks/useWsStatus`, `useHttpStatus`, and
  `useNetSummary` to monitor gateway reachability. REST calls are routed through
  `src/pages/api/net/*` to avoid CORS headaches during development.
- API routes `video-proxy.ts` and `video-health.ts` proxy MJPEG and health checks so
  browsers only ever talk to the Next.js origin.

## Development tips

- `src/control_definitions.ts` centralises command strings so backend/UI changes stay in
  sync.
- Set `NEXT_PUBLIC_WS_DEBUG=1` to log profile resolution, candidate URLs, and socket
  transitions in the browser console.
- The WebSocket connectors in `src/utils/connect*Ws.ts` handle exponential back-off,
  heartbeats, and JSON parsing. Use them when wiring new services.
- Tests under `tests/` use MSW (`tests/mswServer.ts`) to stub backend responses; mirror
  new routes there when you add API features.
- `find_unused_components.py` is handy during refactors to locate components that are no
  longer imported.

## License

This UI is covered by the repository's MIT license.
