#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="interbotix-control-boot.service"
USER_SERVICE_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"

if [[ "$EUID" -eq 0 ]]; then
    printf 'Run this script as the service user, not with sudo.\n' >&2
    exit 1
fi

if [[ ! -f "$REPO_DIR/scripts/$SERVICE_NAME" ]]; then
    printf 'Service file not found: %s\n' "$REPO_DIR/scripts/$SERVICE_NAME" >&2
    exit 1
fi

if ! command -v systemctl >/dev/null 2>&1; then
    printf 'systemctl is required but was not found.\n' >&2
    exit 1
fi

printf 'Installing %s for %s...\n' "$SERVICE_NAME" "$USER"
mkdir -p "$USER_SERVICE_DIR"
chmod +x \
    "$REPO_DIR/scripts/cleanup.sh" \
    "$REPO_DIR/scripts/deploy.sh" \
    "$REPO_DIR/scripts/start-on-boot.sh"
install -m 0644 "$REPO_DIR/scripts/$SERVICE_NAME" "$USER_SERVICE_DIR/$SERVICE_NAME"

systemctl --user disable --now interbotix-control.service 2>/dev/null || true
systemctl --user daemon-reload
systemctl --user enable --now "$SERVICE_NAME"

if command -v loginctl >/dev/null 2>&1; then
    printf 'Enabling user lingering so the service can start without an interactive login...\n'
    sudo loginctl enable-linger "$USER"
fi

printf '\nInstallation complete.\n'
systemctl --user --no-pager --full status "$SERVICE_NAME" || true