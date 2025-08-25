#!/bin/bash

echo "Starting FastAPI as a daemon process..."
cd /home/emackinnon1/InterbotixControlServer

# Use setsid to create a new session completely detached from GitHub Actions
setsid bash -c "
    nohup uv run uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/uvicorn.log 2>&1 &
    echo \$! > /tmp/uvicorn.pid
" >/dev/null 2>&1

# Give it a moment to start
sleep 2

# Verify it started
if [ -f /tmp/uvicorn.pid ] && kill -0 "$(cat /tmp/uvicorn.pid)" 2>/dev/null; then
    echo "FastAPI daemon started successfully with PID $(cat /tmp/uvicorn.pid)"
else
    echo "Failed to start FastAPI daemon - checking logs..."
    if [ -f /tmp/uvicorn.log ]; then
        tail -5 /tmp/uvicorn.log
    fi
    exit 1
fi
