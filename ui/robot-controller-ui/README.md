# Robot Controller UI

The Robot Controller UI is the Next.js frontend for Omega-Code. It provides a control centre
for driving the robot, tuning speed, configuring lighting patterns, monitoring sensors, and
watching the live camera feed. The app targets desktop browsers and talks to the backend via
WebSockets and REST APIs.

## Features

- Real-time drive controls with keyboard/gamepad bindings and speed presets
- Live MJPEG video stream with health checks and reconnect logic
- Lighting composer with colour picker, pattern selection, and timing controls
- Sensor dashboard for ultrasonic distance, line tracker, and location telemetry
- Command log and status indicators so operators can audit recent actions
- Network-aware configuration that switches between local, LAN, and Tailscale backends

## Project structure

The project follows the standard Next.js layout:

| Path | Purpose |
| --- | --- |
| `src/pages/` | Application routes, including API routes that proxy backend services. |
| `src/components/` | UI building blocks (control panels, modals, dashboards, status bars). |
| `src/redux/` | Redux Toolkit slices and store configuration. |
| `src/utils/` | Shared helpers for WebSocket management, environment resolution, and mocks. |
| `src/styles/` | Global styles and Tailwind helpers. |
| `tests/` | Jest unit tests and utilities for component mocking. |
| `cypress/` | End-to-end scenarios (if enabled). |

Images referenced in this README live in `image/README/` at the repository root.

## Prerequisites

- Node.js 18.17+ (Node 20 LTS recommended)
- npm 9+ (or another package manager such as pnpm/yarn if you prefer)

## Setup

1. Install dependencies
   ```bash
   cd ui/robot-controller-ui
   npm install
   ```
2. Configure environment variables
   ```bash
   cp .env.local.example .env.local
   ```
   Update the placeholder hostnames and WebSocket URLs so they point at your backend. At a
   minimum you should set:

   ```ini
   NEXT_PUBLIC_NETWORK_PROFILE=local
   NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT_LOCAL=ws://127.0.0.1:8081/ws/movement
   NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING_LOCAL=ws://127.0.0.1:8082/ws/lighting
   NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC_LOCAL=ws://127.0.0.1:8080/ws/ultrasonic
   NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL=http://127.0.0.1:5000/video_feed
   ```

   The resolver in `src/utils/resolveWsUrl.ts` automatically selects the correct URLs based on
   `NEXT_PUBLIC_NETWORK_PROFILE` (`local`, `lan`, or `tailscale`) and allows runtime overrides
   via query parameters such as `?profile=lan`.

## Running the app

Start the development server and open <http://localhost:3000> in your browser:
```bash
npm run dev
```

For production builds:
```bash
npm run build
npm run start
```

## Testing and linting

Run the available quality checks before opening a pull request:

```bash
npm run lint      # ESLint (Next.js config)
npm test          # Jest unit tests
npx cypress run   # Cypress end-to-end tests (requires backend running)
```

You can launch the Cypress UI locally with `npx cypress open`.

## Development tips

- Use the command log and status bar components to quickly verify which WebSocket profile is
  active. They surface the URLs currently in use.
- Set `NEXT_PUBLIC_WS_DEBUG=1` in `.env.local` to get verbose logging in the browser console
  for WebSocket connection attempts and reconnects.
- The `tests/helpers/wsHarness.ts` utilities make it easy to stub WebSockets when adding new
  components.
- When working offline, set `NEXT_PUBLIC_MOCK_WS=1` to force the UI to use mock data.

## Deployment

When deploying to a Pi or another device, generate a production build (`npm run build`) and
serve it with `npm run start` behind a reverse proxy. Ensure the backend URLs in `.env.local`
(or injected via `window.__ENV__`) use WSS/HTTPS if the site is hosted over TLS.

## License

This UI is covered by the repository's MIT license.
