#!/bin/bash

set -euo pipefail

SERVICE_NAME="interbotix-control.service"
LEGACY_PID_FILE="${LEGACY_UVICORN_PID_FILE:-/tmp/uvicorn.pid}"
LEGACY_PARENT_PID_FILE="${LEGACY_UVICORN_PARENT_PID_FILE:-/tmp/uvicorn_parent.pid}"

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

usage() {
    printf 'Usage: %s [--include-caddy]\n' "${0##*/}" >&2
}

legacy_pid_running() {
    local process_state
    kill -0 "$legacy_pid" 2>/dev/null || return 1
    process_state="$(awk '{ print $3 }' "/proc/$legacy_pid/stat" 2>/dev/null || true)"
    [[ -n "$process_state" && "$process_state" != "Z" ]]
}

include_caddy=false
while (($#)); do
    case "$1" in
        --include-caddy) include_caddy=true ;;
        -h|--help) usage; exit 0 ;;
        *) usage; exit 2 ;;
    esac
    shift
done

if ! systemctl --user show-environment >/dev/null 2>&1; then
    log "ERROR: The user systemd manager is unavailable; cannot verify FastAPI shutdown."
    exit 1
fi

log "Stopping FastAPI service..."
if systemctl --user is-active --quiet "$SERVICE_NAME"; then
    systemctl --user stop "$SERVICE_NAME"
fi
if systemctl --user is-active --quiet "$SERVICE_NAME"; then
    log "ERROR: $SERVICE_NAME is still active."
    exit 1
fi

if [[ -f "$LEGACY_PID_FILE" ]]; then
    legacy_pid="$(<"$LEGACY_PID_FILE")"
    if [[ ! "$legacy_pid" =~ ^[0-9]+$ ]]; then
        log "ERROR: Legacy PID file '$LEGACY_PID_FILE' does not contain a valid PID."
        exit 1
    fi

    if legacy_pid_running; then
        legacy_command="$(tr '\0' ' ' < "/proc/$legacy_pid/cmdline" 2>/dev/null || true)"
        if [[ "$legacy_command" != *uvicorn* || "$legacy_command" != *main:app* ]]; then
            log "ERROR: PID $legacy_pid is not the expected legacy Uvicorn process; leaving it untouched."
            exit 1
        fi

        log "Stopping legacy detached Uvicorn process $legacy_pid..."
        kill -TERM "$legacy_pid"
        for ((attempt = 0; attempt < 50; attempt++)); do
            legacy_pid_running || break
            sleep 0.1
        done
        if legacy_pid_running; then
            log "ERROR: Legacy Uvicorn process $legacy_pid did not stop; leaving its PID file for inspection."
            exit 1
        fi
    fi
    rm -f "$LEGACY_PID_FILE"
fi

rm -f "$LEGACY_PARENT_PID_FILE"

if [[ "$include_caddy" == true ]]; then
    log "Stopping Caddy system service..."
    sudo systemctl stop caddy.service
    if systemctl is-active --quiet caddy.service; then
        log "ERROR: caddy.service is still active."
        exit 1
    fi
fi

log "Cleanup complete. Caddy was left running unless --include-caddy was specified."

