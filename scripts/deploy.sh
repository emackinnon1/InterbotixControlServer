#!/bin/bash

set -e  # Exit on any error

echo "=== InterbotixControlServer Deployment Script ==="

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# Function to cleanup existing services
cleanup_services() {
    ./scripts/cleanup.sh
}

# Function to start Caddy
start_caddy() {
    log "Starting Caddy..."
    if caddy start; then
        log "Caddy started successfully"
    else
        log "ERROR: Failed to start Caddy"
        return 1
    fi
}

# Function to start FastAPI with proper detachment
start_fastapi() {
    log "Starting FastAPI in detached mode..."
    
    # Create a completely detached process using multiple techniques
    (
        # Create new session
        setsid bash -c '
            # Redirect all file descriptors
            exec 0</dev/null
            exec 1>/tmp/uvicorn.log
            exec 2>&1
            
            # Change to the correct directory
            cd /home/emackinnon1/InterbotixControlServer
            
            # Source ROS environment if it exists
            if [ -f workspace/install/setup.bash ]; then
                source workspace/install/setup.bash
            fi
            
            # Start uvicorn and capture PID
            uv run uvicorn main:app --host 0.0.0.0 --port 8000 &
            echo $! > /tmp/uvicorn.pid
            
            # Wait for the background process
            wait
        ' &
    ) &
    
    # Give it time to start
    sleep 5
    
    # Verify it started
    if [ -f /tmp/uvicorn.pid ] && kill -0 "$(cat /tmp/uvicorn.pid)" 2>/dev/null; then
        log "FastAPI started successfully with PID $(cat /tmp/uvicorn.pid)"
        return 0
    else
        log "ERROR: FastAPI failed to start"
        if [ -f /tmp/uvicorn.log ]; then
            log "Last few lines of uvicorn log:"
            tail -10 /tmp/uvicorn.log
        fi
        return 1
    fi
}

# Function to verify services are running
verify_services() {
    log "Verifying services..."
    
    # Check Caddy
    if pgrep -f "caddy" > /dev/null; then
        log "✓ Caddy is running"
    else
        log "✗ Caddy is not running"
        return 1
    fi
    
    # Check FastAPI
    if pgrep -f "uvicorn main:app" > /dev/null; then
        log "✓ FastAPI is running"
    else
        log "✗ FastAPI is not running"
        return 1
    fi
    
    # Check HTTP response
    if curl -s -f http://localhost:8000/ > /dev/null; then
        log "✓ FastAPI is responding to HTTP requests"
    else
        log "✗ FastAPI is not responding to HTTP requests"
        return 1
    fi
    
    log "All services verified successfully!"
    return 0
}

# Main deployment process
main() {
    log "Starting deployment process..."
    
    # Ensure we're in the right directory
    cd /home/emackinnon1/InterbotixControlServer
    
    # Step 1: Cleanup
    cleanup_services
    
    # Step 2: Start Caddy
    start_caddy
    
    # Step 3: Start FastAPI
    start_fastapi
    
    # Step 4: Verify everything is working
    verify_services
    
    log "Deployment completed successfully!"
}

# Run main function
main "$@"
