#!/usr/bin/env bash

set -euo pipefail

DEPLOY_REPO_DIR="${DEPLOY_REPO_DIR:-/home/emackinnon1/InterbotixControlServer}"
SOURCE_REPO_DIR="${GITHUB_WORKSPACE:-}"
DEPLOY_SHA="${GITHUB_SHA:-}"
EXPECTED_REMOTE="https://github.com/emackinnon1/InterbotixControlServer.git"

if [[ "$(id -un)" != "emackinnon1" || "$HOME" != "/home/emackinnon1" ]]; then
    printf 'Production checkout updates must run as emackinnon1 with HOME=/home/emackinnon1.\n' >&2
    exit 1
fi
if [[ "${GITHUB_EVENT_NAME:-}" != "push" || "${GITHUB_REF:-}" != "refs/heads/main" ]]; then
    printf 'Deployment checkout updates are allowed only for pushes to main.\n' >&2
    exit 1
fi
if [[ ! -d "$SOURCE_REPO_DIR" || -z "$DEPLOY_SHA" ]]; then
    printf 'GITHUB_WORKSPACE and GITHUB_SHA are required.\n' >&2
    exit 1
fi
if ! git -C "$DEPLOY_REPO_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    printf 'Deployment directory is not a Git worktree: %s\n' "$DEPLOY_REPO_DIR" >&2
    exit 1
fi
if [[ "$(git -C "$DEPLOY_REPO_DIR" remote get-url origin)" != "$EXPECTED_REMOTE" ]]; then
    printf 'Unexpected origin for deployment checkout: %s\n' "$DEPLOY_REPO_DIR" >&2
    exit 1
fi
if [[ "$(git -C "$SOURCE_REPO_DIR" rev-parse HEAD)" != "$DEPLOY_SHA" ]]; then
    printf 'Actions checkout does not match the requested commit %s.\n' "$DEPLOY_SHA" >&2
    exit 1
fi

local_changes="$(git -C "$DEPLOY_REPO_DIR" status --porcelain --untracked-files=all)"
if [[ -n "$local_changes" ]]; then
    printf 'Deployment checkout has local changes; refusing to overwrite them:\n%s\n' "$local_changes" >&2
    exit 1
fi

git -C "$DEPLOY_REPO_DIR" fetch --quiet "$SOURCE_REPO_DIR" "$DEPLOY_SHA"
git -C "$DEPLOY_REPO_DIR" update-ref refs/remotes/origin/main "$DEPLOY_SHA"
git -C "$DEPLOY_REPO_DIR" switch --quiet --force-create main FETCH_HEAD

if [[ "$(git -C "$DEPLOY_REPO_DIR" rev-parse HEAD)" != "$DEPLOY_SHA" ]]; then
    printf 'Deployment checkout did not reach requested commit %s.\n' "$DEPLOY_SHA" >&2
    exit 1
fi

printf 'Deployment checkout updated to %s.\n' "$DEPLOY_SHA"