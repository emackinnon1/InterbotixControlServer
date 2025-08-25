#!/bin/bash

echo "Cleaning up any existing services and processes..."

echo "Stopping Caddy..."
caddy stop 2>/dev/null || true

echo "Stopping uvicorn processes..."
pkill -f "uvicorn main:app" 2>/dev/null || true

echo "Stopping any remaining caddy processes..."
pkill -f "caddy" 2>/dev/null || true

echo "Cleaning up PID files..."
rm -f /tmp/uvicorn.pid

echo "Waiting for processes to terminate..."
sleep 2

echo "Cleanup complete."
