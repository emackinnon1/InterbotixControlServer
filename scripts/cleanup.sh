#!/bin/bash

echo "Cleaning up any existing services and processes..."

# Stop caddy gracefully first, then forcefully if needed
echo "Stopping Caddy..."
(timeout 3 caddy stop 2>/dev/null || pkill -9 caddy 2>/dev/null) || true

# Stop uvicorn processes
echo "Stopping uvicorn processes..."
pkill -f "uvicorn main:app" 2>/dev/null || true

# Clean up PID files
echo "Cleaning up PID files..."
rm -f /tmp/uvicorn.pid

echo "Cleanup complete."
