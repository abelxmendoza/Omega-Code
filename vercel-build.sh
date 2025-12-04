#!/bin/bash
set -e

echo "⚙️  Starting Omega-Tech build..."
echo "   Node: $(node -v)"
echo "   NPM:  $(npm -v)"
echo "   Working directory: $(pwd)"

# Debug: Show current directory and contents
echo "Directory contents:"
ls -la

# Navigate to UI directory
if [ ! -d "ui/robot-controller-ui" ]; then
  echo "❌ UI directory missing!"
  echo "Looking for ui directory..."
  if [ -d "ui" ]; then
    echo "ui directory exists, contents:"
    ls -la ui/
  else
    echo "ui directory does not exist!"
  fi
  exit 1
fi

cd ui/robot-controller-ui

# Verify package.json exists
if [ ! -f "package.json" ]; then
  echo "❌ package.json not found in ui/robot-controller-ui!"
  exit 1
fi

# Install dependencies
echo "Installing dependencies..."
npm ci || (echo "❌ NPM install failed"; exit 1)

# Build
echo "Building Next.js app..."
npm run build || (echo "❌ NEXT.JS build failed"; exit 1)

echo "✅ Build completed successfully."

