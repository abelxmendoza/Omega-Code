#!/usr/bin/env node

/**
 * Pre-build check script for Vercel deployments
 * This script verifies dependencies and provides helpful error messages
 */

const fs = require('fs');
const path = require('path');

console.log('[Vercel Build Debug] Starting pre-build checks...');
console.log('[Vercel Build Debug] Node version:', process.version);
console.log('[Vercel Build Debug] Platform:', process.platform);
console.log('[Vercel Build Debug] Architecture:', process.arch);
console.log('[Vercel Build Debug] Current directory:', process.cwd());
console.log('[Vercel Build Debug] NODE_ENV:', process.env.NODE_ENV);
console.log('[Vercel Build Debug] VERCEL_ENV:', process.env.VERCEL_ENV);

// Check if node_modules exists
const nodeModulesPath = path.join(process.cwd(), 'node_modules');
if (!fs.existsSync(nodeModulesPath)) {
  console.error('[Vercel Build Error] node_modules directory not found!');
  console.error('[Vercel Build Error] Please ensure npm install has been run.');
  process.exit(1);
}

// Check for critical dependencies
const packageJsonPath = path.join(process.cwd(), 'package.json');
if (!fs.existsSync(packageJsonPath)) {
  console.error('[Vercel Build Error] package.json not found!');
  process.exit(1);
}

const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
const criticalDeps = ['next', 'react', 'react-dom', 'typescript'];

console.log('[Vercel Build Debug] Checking critical dependencies...');
for (const dep of criticalDeps) {
  const depPath = path.join(nodeModulesPath, dep);
  if (!fs.existsSync(depPath)) {
    console.error(`[Vercel Build Error] Critical dependency '${dep}' not found in node_modules!`);
    console.error(`[Vercel Build Error] Expected path: ${depPath}`);
    console.error(`[Vercel Build Error] Please run: npm install`);
    process.exit(1);
  }
  console.log(`[Vercel Build Debug] ✓ ${dep} found`);
}

// Check for ESLint (optional but helpful)
const eslintPath = path.join(nodeModulesPath, 'eslint');
if (!fs.existsSync(eslintPath)) {
  console.warn('[Vercel Build Warning] ESLint not found in node_modules.');
  console.warn('[Vercel Build Warning] ESLint is required for Next.js linting during builds.');
  console.warn('[Vercel Build Warning] Install with: npm install --save-dev eslint');
  console.warn('[Vercel Build Warning] Continuing build anyway...');
} else {
  console.log('[Vercel Build Debug] ✓ eslint found');
}

// Check for Cypress (optional - only needed for e2e tests)
const cypressPath = path.join(nodeModulesPath, 'cypress');
if (!fs.existsSync(cypressPath)) {
  console.warn('[Vercel Build Warning] Cypress not found in node_modules.');
  console.warn('[Vercel Build Warning] This is expected if cypress.config.ts is excluded from TypeScript compilation.');
  console.warn('[Vercel Build Warning] If you see TypeScript errors about Cypress, ensure cypress.config.ts is excluded in tsconfig.json');
} else {
  console.log('[Vercel Build Debug] ✓ cypress found');
}

// Check TypeScript config
const tsconfigPath = path.join(process.cwd(), 'tsconfig.json');
if (fs.existsSync(tsconfigPath)) {
  const tsconfig = JSON.parse(fs.readFileSync(tsconfigPath, 'utf8'));
  const excludes = tsconfig.exclude || [];
  if (!excludes.includes('cypress.config.ts')) {
    console.warn('[Vercel Build Warning] cypress.config.ts is not excluded in tsconfig.json');
    console.warn('[Vercel Build Warning] This may cause TypeScript errors if Cypress is not installed.');
    console.warn('[Vercel Build Warning] Add "cypress.config.ts" to the exclude array in tsconfig.json');
  } else {
    console.log('[Vercel Build Debug] ✓ cypress.config.ts is excluded from TypeScript compilation');
  }
}

// Check Next.js config
const nextConfigPath = path.join(process.cwd(), 'next.config.mjs');
if (fs.existsSync(nextConfigPath)) {
  console.log('[Vercel Build Debug] ✓ next.config.mjs found');
} else {
  console.warn('[Vercel Build Warning] next.config.mjs not found');
}

console.log('[Vercel Build Debug] Pre-build checks completed successfully!');
console.log('[Vercel Build Debug] Proceeding with build...');

