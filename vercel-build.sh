#!/bin/bash
set -e

# Navigate to UI directory
if [ ! -d "ui/robot-controller-ui" ]; then
  echo "Error: ui/robot-controller-ui directory not found!"
  echo "Current directory: $(pwd)"
  echo "Contents:"
  ls -la
  exit 1
fi

cd ui/robot-controller-ui

# Install dependencies
echo "Installing dependencies..."
npm install --verbose

# Build
echo "Building Next.js app..."
npm run build

