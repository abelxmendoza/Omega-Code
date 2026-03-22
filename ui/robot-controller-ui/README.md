# Robot Controller UI

Next.js frontend for Omega-Code. Provides a control centre for driving the rover, tuning servos and speed, orchestrating lighting scenes, monitoring sensors, and managing connectivity. Talks to the backend via WebSockets, REST APIs, and the gateway proxy.

## Features

- Real-time drive controls with keyboard/gamepad bindings, timed moves, and servo nudges.
- Live MJPEG video feed with placeholder frames and health polling.
- Lighting composer with colour selection, pattern/mode controls, and brightness sliders.
- Music-reactive lighting mode that listens for microphone audio (with a graceful fallback when no input device is present).
- Sensor dashboards for ultrasonic distance, line tracker state, and GPS data.
- Autonomy modal for starting/stopping modes exposed by the backend controller.
- Header service bar showing link health and gateway profile, backed by REST/WS checks.
- Network wizard (`OmegaNetworkWizard`) that scans Wi-Fi, triggers PAN helpers, and exposes active URLs.
- ROS2 management page for container control, topic inspection, and log tailing.
- Settings page with per-section config editors, hardware map viewer, and import/export.
- PWA — installable on desktop and mobile with offline caching via service worker.

## Project structure

| Path | Purpose |
| --- | --- |
| `src/pages/` | Application routes (`index`, `network`, `ros`, `services`, `settings`), custom `_app`, error pages, and API routes that proxy backend services (`api/net/*`, `api/video-*`, `api/performance-proxy/*`, `api/system/mode/*`). |
| `src/components/` | UI building blocks organised by domain (see table below). |
| `src/context/` | React contexts — `CommandContext` (WebSocket + command log), `MacroContext` (macro editor + runtime), `CapabilityContext` (feature detection). |
| `src/hooks/` | Custom hooks for WebSocket lifecycles, HTTP polling, network summaries, config loading, ROS2 status, and gamepad input. |
| `src/utils/` | Network profile resolvers, WebSocket connectors (`connect*Ws.ts`), debounce/throttle, autonomy API client, offline guard (`robotFetch`), and profile persistence. |
| `src/config/` | Centralised configuration — `gateway.ts` for profile-aware URL resolution, `environment.ts` for env var handling, `mobileOptimization.ts`. |
| `src/constants/` | Enumerations such as service status badges and macro command names. |
| `src/themes/` | Theme tokens — `omega-theme.ts` (dark industrial palette), `cyber-theme.ts` (neon purple). |
| `src/control_definitions.ts` | Single source of truth for all WebSocket command strings. |
| `src/lib/` | Small shared utilities (date/time helpers, shadcn/ui `cn()`). |
| `src/styles/` | Global Tailwind SCSS and CSS variable theme tokens. |
| `tests/` | Jest + Testing Library suites (90 passing) with MSW handlers, mocks, and helpers. |
| `cypress/` | Cypress end-to-end specs (headless by default; UI runner via `npx cypress open`). |
| `docs/` | Component reference (`components.md`). |
| `scripts/find_unused_components.py` | Utility for auditing unused React components during refactors. |

### Component subdirectories

| Folder | What lives here |
| --- | --- |
| `control/` | `CarControlPanel`, `CameraControlPanel`, `SpeedControl`, `ControlButtons`, `AutonomyModal`, `XboxControllerStatus`, `MotorTelemetryPanel`, `ServoTelemetryPanel`, `EnhancedServoTelemetryPanel`, `MovementV2Modal` |
| `lighting/` | `LedControl`, `LedModal`, `LightingMode`, `LightingPattern`, `ColorWheel` |
| `sensors/` | `SensorDashboard`, `LineTrackerStatus`, `UltrasonicSensorStatus`, `UltrasonicVisualization` |
| `network/` | `OmegaNetworkWizard` (Wi-Fi scan, AP mode, PAN helpers, active URL display) |
| `ros/` | `ROSManagementPanel`, `RobotController`, `ROS2TopicViewer`, `AutonomousActions`, `CameraViewer`, `MapViewer`, `TelemetryVisualization` |
| `settings/` | Per-section config editors (`MovementConfigEditor`, `CameraConfigEditor`, `LightingConfigEditor`, `NetworkConfigEditor`), `HardwareMapViewer`, `ConfigImportExport`, `ApplyRestartServices`, `ServiceAutostartEditor`, `ProfileSelector` |
| `services/` | `ServiceTable`, `ServiceLogs` |
| `capability/` | `CapabilityGate`, `CapabilityStatus`, `CapabilityInfoModal`, `FeatureTooltip` |
| `macros/` | `MacroEditor`, `MacroManager` |
| `common/` | `StatusDot` and other shared primitives |
| `ui/` | shadcn/ui primitives (`Button`, `Badge`, `Card`, `Dialog`, `Input`, `Label`, `Select`, `Slider`, `Switch`) |

Images referenced in the documentation live in `image/README/` at the repository root.

## Prerequisites

- Node.js 18.17+ (Node 20 LTS recommended)
- npm 9+

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

The UI uses a **centralised gateway configuration system** (`src/config/gateway.ts`) that automatically resolves URLs based on the active network profile. All API routes use this config for consistent profile-aware URL resolution.

Key variables:

| Variable | Purpose |
| --- | --- |
| `NEXT_PUBLIC_NETWORK_PROFILE` | Default profile (`local`, `lan`, `tailscale`). Persisted in `localStorage` and `?profile` query param. |
| `NEXT_PUBLIC_ROBOT_HOST_*` | Robot host address per profile (takes precedence over `GATEWAY_HOST_*`). |
| `NEXT_PUBLIC_GATEWAY_HOST` / `_PORT` | FastAPI gateway base host + port (fallback). |
| `NEXT_PUBLIC_VIDEO_STREAM_URL_*` | Per-profile MJPEG endpoint for `/api/video-proxy` and the camera component. |
| `NEXT_PUBLIC_BACKEND_WS_URL_*` | Per-profile WebSocket URLs for movement, lighting, ultrasonic, line tracker, and location. |
| `NEXT_PUBLIC_WS_FORCE_INSECURE` | Set to `1` to keep `ws://` on HTTPS pages (when a reverse proxy handles TLS). |
| `NEXT_PUBLIC_WS_DEBUG` | Set to `1` for verbose console logs on profile resolution and socket events. |
| `NEXT_PUBLIC_ROBOT_ENABLED` | Set to `false` to run in offline/demo mode (blocks all fetch and WebSocket calls). |

See `.env.local.example` for the full list with documentation.

## Running the app

```bash
npm run dev       # Development server — http://localhost:3000
npm run build     # Production build
npm run start     # Serve the production build
```

You can also use the root Makefile to export the correct profile automatically:

```bash
make ui-dev PROFILE=lan
```

## Testing and linting

```bash
npm run lint      # ESLint (Next.js + Tailwind rules)
npm test          # Jest unit/component tests (90 tests, 0 failures)
npx cypress run   # Cypress E2E tests (requires backend/gateway running)
npx cypress open  # Interactive Cypress runner
```

The Jest suite uses MSW (`tests/mswServer.ts`) to stub all backend responses — no live backend needed. Mirror new routes in the MSW handlers when you add API features.

## Network tooling

- **`OmegaNetworkWizard`** (`src/components/network/OmegaNetworkWizard.tsx`) — opened from the Header. Opens a dedicated WebSocket, exposes Wi-Fi scan, AP mode toggle, and PAN connect actions. Respects the active network profile from `localStorage`.
- **Header status bar** — uses `useWsStatus`, `useHttpStatus`, and `useNetSummary` to show live pill indicators for each backend service.
- **API routes** `src/pages/api/net/*` — proxy gateway REST calls to avoid browser CORS restrictions during development.
- **`video-proxy.ts` / `video-health.ts`** — proxy the MJPEG stream and health endpoint so the browser only talks to the Next.js origin.

## Development tips

- `src/control_definitions.ts` centralises all command strings — change once, works everywhere.
- Set `NEXT_PUBLIC_WS_DEBUG=1` to log profile resolution, candidate URLs, and socket transitions in the browser console.
- The five `src/utils/connect*Ws.ts` connectors (`connectMovementWs`, `connectLightingWs`, `connectUltrasonicWs`, `connectLineTrackerWs`, `connectLocationWs`) handle exponential back-off, heartbeats, and JSON parsing. Use them as a template when wiring new services.
- Use `buildGatewayUrl()` from `src/config/gateway.ts` for all gateway REST calls to get consistent profile-aware URL resolution.
- `src/utils/network.ts` exports `robotFetch` which returns `RobotResponse` — check `response.offline` to detect when the robot backend is unreachable without a type cast.
- `scripts/find_unused_components.py` is handy after refactors to catch components that are no longer imported.

## PWA / Install

The UI is installable as a Progressive Web App.

- **Desktop (Chrome/Edge)**: click the Install icon in the address bar.
- **iOS (Safari)**: Share → Add to Home Screen.
- **Android (Chrome)**: menu → Install app.

The header shows a download icon when installation is available. A service worker at `/sw.js` handles basic offline caching. Edit `/public/manifest.json` to change the app name, theme colour, or icons.

## License

This UI is covered by the repository's MIT license.
