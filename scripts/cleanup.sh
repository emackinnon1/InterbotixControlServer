#!/bin/bash

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "Cleaning up existing services..."

# Stop systemd service if it exists
if systemctl --user is-active --quiet interbotix-control.service 2>/dev/null; then
    log "Stopping systemd service..."
    systemctl --user stop interbotix-control.service || true
fi

# Stop Caddy gracefully
log "Stopping Caddy..."
timeout 5 caddy stop 2>/dev/null || pkill -9 caddy 2>/dev/null || true

# Stop any uvicorn processes
log "Stopping uvicorn processes..."
pkill -f "uvicorn main:app" 2>/dev/null || true
pkill -f "uv run uvicorn" 2>/dev/null || true

# Clean up PID files
log "Cleaning up PID files..."
rm -f /tmp/uvicorn.pid /tmp/uvicorn_parent.pid

# Wait a moment for processes to fully terminate
sleep 2

log "Cleanup complete"

