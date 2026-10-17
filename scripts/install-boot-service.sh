#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="interbotix-control.service"
LEGACY_SERVICE_NAME="interbotix-control-boot.service"
USER_SERVICE_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
CADDY_CONFIG="/etc/caddy/Caddyfile"

if [[ "$EUID" -eq 0 ]]; then
    printf 'Run this script as the service user, not with sudo.\n' >&2
    exit 1
fi
if [[ "$(id -un)" != "emackinnon1" || "$HOME" != "/home/emackinnon1" ]]; then
    printf 'Install the service as emackinnon1 with HOME=/home/emackinnon1.\n' >&2
    exit 1
fi

if [[ ! -f "$REPO_DIR/scripts/$SERVICE_NAME" ]]; then
    printf 'Service file not found: %s\n' "$REPO_DIR/scripts/$SERVICE_NAME" >&2
    exit 1
fi
if [[ ! -f "$REPO_DIR/Caddyfile" ]]; then
    printf 'Caddyfile not found: %s/Caddyfile\n' "$REPO_DIR" >&2
    exit 1
fi

if ! command -v systemctl >/dev/null 2>&1; then
    printf 'systemctl is required but was not found.\n' >&2
    exit 1
fi
if ! systemctl --user show-environment >/dev/null 2>&1; then
    printf 'The user systemd manager is unavailable; log in as the service user and retry.\n' >&2
    exit 1
fi
if ! command -v sudo >/dev/null 2>&1; then
    printf 'sudo is required to configure the system Caddy service.\n' >&2
    exit 1
fi
if ! command -v loginctl >/dev/null 2>&1; then
    printf 'loginctl is required to enable the user service at boot.\n' >&2
    exit 1
fi
if ! sudo loginctl enable-linger "$USER"; then
    printf 'Failed to enable user lingering for %s; the service may not start at boot.\n' "$USER" >&2
    exit 1
fi

printf 'Installing %s for %s...\n' "$SERVICE_NAME" "$USER"
mkdir -p "$USER_SERVICE_DIR"
chmod +x \
    "$REPO_DIR/scripts/cleanup.sh" \
    "$REPO_DIR/scripts/deploy.sh" \
    "$REPO_DIR/scripts/run-server.sh" \
    "$REPO_DIR/scripts/update-deployment-checkout.sh"
install -m 0644 "$REPO_DIR/scripts/$SERVICE_NAME" "$USER_SERVICE_DIR/$SERVICE_NAME"
systemctl --user daemon-reload

if ! systemctl cat caddy.service >/dev/null 2>&1; then
    printf 'The package-managed caddy.service was not found. Install Caddy before continuing.\n' >&2
    exit 1
fi

caddy_backup=""
caddy_config_changed=false
restore_caddy_config() {
    if [[ "$caddy_config_changed" != true ]]; then
        return 0
    fi
    if [[ -n "$caddy_backup" && -f "$caddy_backup" ]]; then
        sudo cp -a "$caddy_backup" "$CADDY_CONFIG"
    else
        sudo rm -f "$CADDY_CONFIG"
    fi
}

if ! cmp -s "$REPO_DIR/Caddyfile" "$CADDY_CONFIG"; then
    caddy_backup="$CADDY_CONFIG.before-interbotix.$(date -u +%Y%m%dT%H%M%SZ)"
    if [[ -f "$CADDY_CONFIG" ]]; then
        sudo cp -a "$CADDY_CONFIG" "$caddy_backup"
    fi
    caddy_config_changed=true
    if ! sudo install -D -m 0644 "$REPO_DIR/Caddyfile" "$CADDY_CONFIG"; then
        restore_caddy_config
        printf 'Failed to install the project Caddyfile; the previous config was restored.\n' >&2
        exit 1
    fi
fi

if ! sudo /usr/bin/caddy validate --config "$CADDY_CONFIG"; then
    restore_caddy_config
    printf 'Caddy configuration validation failed; the previous config was restored.\n' >&2
    exit 1
fi

sudo systemctl enable caddy.service
if sudo systemctl is-active --quiet caddy.service; then
    if ! sudo systemctl reload caddy.service; then
        restore_caddy_config
        sudo systemctl reload caddy.service || true
        printf 'Failed to reload caddy.service. Check its journal before retrying.\n' >&2
        exit 1
    fi
else
    if ! sudo systemctl start caddy.service; then
        restore_caddy_config
        sudo systemctl start caddy.service || true
        printf 'Failed to start caddy.service. Check its journal before retrying.\n' >&2
        exit 1
    fi
fi
if ! sudo systemctl is-active --quiet caddy.service; then
    restore_caddy_config
    printf 'caddy.service is not active after configuration.\n' >&2
    exit 1
fi

systemctl --user stop "$LEGACY_SERVICE_NAME" 2>/dev/null || true
bash "$REPO_DIR/scripts/cleanup.sh"
bash "$REPO_DIR/scripts/deploy.sh"
systemctl --user enable "$SERVICE_NAME"

systemctl --user disable --now "$LEGACY_SERVICE_NAME" 2>/dev/null || true
rm -f "$USER_SERVICE_DIR/$LEGACY_SERVICE_NAME"
systemctl --user daemon-reload

printf '\nInstallation complete.\n'
systemctl --user --no-pager --full status "$SERVICE_NAME"
sudo systemctl --no-pager --full status caddy.service