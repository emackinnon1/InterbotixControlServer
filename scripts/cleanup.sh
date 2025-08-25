#!/bin/bash

echo "Cleaning up any existing services and processes..."

# Stop systemd service first
echo "Stopping FastAPI systemd service..."
systemctl --user stop interbotix-control.service 2>/dev/null || true

# Stop caddy gracefully first, then forcefully if needed
echo "Stopping Caddy..."
(timeout 3 caddy stop 2>/dev/null || pkill -9 caddy 2>/dev/null) || true

# Stop any remaining uvicorn processes
echo "Stopping any remaining uvicorn processes..."
pkill -f "uvicorn main:app" 2>/dev/null || true

# Clean up PID files
echo "Cleaning up PID files..."
rm -f /tmp/uvicorn.pid

echo "Cleanup complete."

echo "Cleanup complete."
