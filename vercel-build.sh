#!/bin/bash
set -e

# Debug: Show current directory and contents
echo "Current directory: $(pwd)"
echo "Directory contents:"
ls -la

# Navigate to UI directory
if [ ! -d "ui/robot-controller-ui" ]; then
  echo "Error: ui/robot-controller-ui directory not found!"
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
  echo "Error: package.json not found in ui/robot-controller-ui!"
  exit 1
fi

# Install dependencies
echo "Installing dependencies..."
npm install --verbose

# Build
echo "Building Next.js app..."
npm run build

