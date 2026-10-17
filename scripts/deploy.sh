#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="interbotix-control.service"
HEALTH_URL="${HEALTH_URL:-http://127.0.0.1:8000/}"
HEALTH_TIMEOUT="${HEALTH_TIMEOUT:-60}"
HEALTH_INTERVAL="${HEALTH_INTERVAL:-2}"

log() {
    printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$1"
}

if [[ "$EUID" -eq 0 ]]; then
    printf 'Run deployment as the service user, not with sudo.\n' >&2
    exit 1
fi
if [[ "$(id -un)" != "emackinnon1" || "$HOME" != "/home/emackinnon1" ]]; then
    printf 'Deployment must run as emackinnon1 with HOME=/home/emackinnon1.\n' >&2
    exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
    printf 'uv is required on PATH to sync the application environment.\n' >&2
    exit 1
fi

cd "$REPO_DIR"
log "Syncing application dependencies..."
uv sync --locked --python /usr/bin/python3

log "Stopping the current FastAPI service..."
bash "$SCRIPT_DIR/cleanup.sh"

log "Starting $SERVICE_NAME..."
systemctl --user start "$SERVICE_NAME"

deadline=$((SECONDS + HEALTH_TIMEOUT))
while ((SECONDS <= deadline)); do
    if curl --fail --silent --max-time 2 --output /dev/null "$HEALTH_URL"; then
        log "FastAPI is healthy at $HEALTH_URL."
        exit 0
    fi
    if ! systemctl --user is-active --quiet "$SERVICE_NAME"; then
        break
    fi
    sleep "$HEALTH_INTERVAL"
done

log "ERROR: FastAPI did not become healthy at $HEALTH_URL."
journalctl --user -u "$SERVICE_NAME" --since '-2 minutes' --no-pager >&2 || true
exit 1
