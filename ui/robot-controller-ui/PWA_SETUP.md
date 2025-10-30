# Progressive Web App (PWA) Setup

The Omega Robot Controller UI is now installable as a Progressive Web App (PWA) on desktop and mobile devices.

## Features

✅ **Installable** - Add to home screen or desktop
✅ **Offline Support** - Basic caching for offline access
✅ **Standalone Mode** - Runs in its own window without browser UI
✅ **App Icons** - Uses Omega Tech logo for app icon
✅ **Theme Color** - Cyber-purple theme color (#C400FF)

## Installation

### Desktop (Chrome/Edge)

1. Visit the app in your browser
2. Click the **Install** button in the address bar (or use the Download icon in the header)
3. Click "Install" in the prompt
4. The app will launch in its own window

### Mobile (iOS)

1. Open the app in Safari
2. Tap the **Share** button
3. Scroll down and tap **"Add to Home Screen"**
4. Tap "Add" to confirm
5. The app icon will appear on your home screen

### Mobile (Android)

1. Open the app in Chrome
2. Tap the **menu** (three dots)
3. Tap **"Install app"** or **"Add to Home screen"**
4. Tap "Install" to confirm
5. The app icon will appear on your home screen

## Manual Install Button

The header includes a purple Download icon that appears when the app is installable. Click it to trigger the installation prompt.

## Service Worker

The app includes a service worker (`/sw.js`) that:
- Caches the main app files
- Provides basic offline functionality
- Automatically registers on page load

## Customization

### Icons

Replace the icon files in `/public/image/README/omegatechlogopro-noBackground.png` or generate proper PWA icons:

```bash
# Generate 192x192 and 512x512 icons from your logo
# Update manifest.json with the new icon paths
```

### Manifest

Edit `/public/manifest.json` to customize:
- App name and description
- Theme colors
- Display mode
- App shortcuts

## Testing

1. **Local Testing**: Run `npm run dev` and test installation in Chrome DevTools
2. **HTTPS Required**: PWAs require HTTPS (automatically provided by Vercel in production)
3. **Service Worker**: Check DevTools > Application > Service Workers to verify registration

## Troubleshooting

- **Install button not showing**: App may already be installed, or browser doesn't support PWA
- **Offline not working**: Service worker may not be registered - check console
- **Icons missing**: Ensure icon files exist in `/public/` directory

## Next Steps

- [ ] Generate proper 192x192 and 512x512 icon files
- [ ] Add more offline caching strategies
- [ ] Implement background sync for robot commands
- [ ] Add push notifications for robot alerts

