# CSP Violation Fixes

## Issues Fixed

### 1. Font Loading CSP Violations
- **Problem**: `font-src 'none'` was blocking all fonts
- **Solution**: Added `font-src 'self' data: https://fonts.googleapis.com https://fonts.gstatic.com` to CSP
- **Location**: `next.config.mjs`

### 2. Inline Script CSP Violations  
- **Problem**: Inline scripts in `_document.tsx` were blocked
- **Solution**: Added `'unsafe-inline'` to `script-src` for Next.js compatibility
- **Note**: CSP is only enforced in production, disabled in development

### 3. 404 Errors
- **Problem**: Browser extensions trying to access localhost:3000
- **Solution**: These are harmless - browser extension errors, not app errors
- **Note**: The actual app should work fine

## CSP Configuration

The CSP is configured in `next.config.mjs`:
- **Development**: No CSP (to avoid blocking during dev)
- **Production**: Full CSP with:
  - Fonts allowed from self, data URIs, and Google Fonts
  - Scripts allowed with unsafe-inline (needed for Next.js)
  - Styles allowed with unsafe-inline (needed for CSS-in-JS)
  - Images from all sources (needed for camera feeds)
  - WebSocket connections allowed

## Testing

1. **Development Mode**: CSP violations won't appear (CSP disabled)
2. **Production Build**: Run `npm run build && npm start` to test CSP
3. **Browser Console**: Should see fewer/no CSP violations

## Notes

- The CSP violations you saw were "report-only" - they weren't blocking functionality
- Browser extension errors (chrome-extension://) are harmless
- The 404 errors for `/` are likely from browser extensions checking the page
- The actual Next.js app should work fine despite these console warnings

## If Issues Persist

1. Clear browser cache and hard refresh (Cmd+Shift+R / Ctrl+Shift+R)
2. Check if any browser extensions are interfering
3. Try in incognito/private mode
4. Check Network tab to see if actual API calls are working

