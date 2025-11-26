# Vercel Build Debugging Enhancements

This document explains the changes made to improve error messages and debugging for Vercel builds.

## Changes Made

### 1. TypeScript Configuration (`ui/robot-controller-ui/tsconfig.json`)
- **Added exclusion**: `cypress.config.ts` and `cypress/**/*` are now excluded from TypeScript compilation
- **Why**: Prevents TypeScript errors when Cypress is not installed during builds (Cypress is a devDependency and may not be available in all build environments)

### 2. Next.js Configuration (`ui/robot-controller-ui/next.config.mjs`)
- **Added verbose logging**: Build ID, server status, and environment variables are logged during builds
- **Added Cypress alias**: Webpack now ignores Cypress imports to prevent build failures
- **Enhanced error handling**: Better configuration for ESLint and TypeScript error reporting

### 3. Vercel Configuration (`vercel.json`)
- **Set root directory**: `rootDirectory` is now set to `ui/robot-controller-ui` so Vercel knows where the Next.js app is located
- **Added verbose flags**: `--verbose` flag added to npm install commands for better debugging
- **Added environment variables**: `VERCEL_DEBUG=1` and `NODE_ENV=production` set for better logging

### 4. Pre-Build Check Script (`ui/robot-controller-ui/scripts/pre-build-check.js`)
- **Dependency verification**: Checks for critical dependencies (next, react, react-dom, typescript)
- **Environment logging**: Logs Node version, platform, architecture, and environment variables
- **Helpful error messages**: Provides specific error messages with suggested fixes
- **Warning system**: Warns about missing optional dependencies (ESLint, Cypress) without failing the build

### 5. Package.json Scripts (`ui/robot-controller-ui/package.json`)
- **Added prebuild script**: Automatically runs pre-build checks before every build
- **Added build:verbose script**: Alternative build command with trace warnings for debugging

## How It Works

1. **Pre-build Phase**: The `prebuild` script runs automatically before `npm run build`
   - Checks for critical dependencies
   - Logs environment information
   - Provides helpful error messages if something is missing

2. **Build Phase**: Next.js build runs with enhanced logging
   - Webpack logs build information
   - TypeScript compilation excludes Cypress config
   - ESLint runs with proper error reporting

3. **Error Messages**: If something fails, you'll see:
   - Clear error messages indicating what's missing
   - Suggested commands to fix the issue
   - Environment information to help debug platform-specific issues

## Common Issues and Solutions

### Issue: "ESLint must be installed"
**Solution**: ESLint is in devDependencies. Ensure `npm install` runs with devDependencies (default behavior).

### Issue: "Cannot find module 'cypress'"
**Solution**: This should be fixed by excluding `cypress.config.ts` from TypeScript compilation. If it persists, ensure Cypress is in devDependencies.

### Issue: Build fails silently
**Solution**: Check the build logs for `[Vercel Build Debug]` messages. These provide detailed information about what's happening.

## Testing Locally

To test the build process locally:

```bash
cd ui/robot-controller-ui
npm run build
```

To test with verbose output:

```bash
cd ui/robot-controller-ui
npm run build:verbose
```

## Vercel-Specific Notes

- Vercel automatically installs dependencies before building
- The `rootDirectory` setting tells Vercel where your Next.js app is located
- The prebuild script runs automatically via npm lifecycle hooks
- All debug messages are prefixed with `[Vercel Build Debug]` for easy filtering

