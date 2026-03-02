#!/bin/bash
# Git credential helper for novarover: supplies GitHub PAT from GITHUB_TOKEN env var.
# Only responds for host=github.com. Set GITHUB_TOKEN in .env or your shell profile.
# See docs/GIT_PUSH_SETUP.md for setup.

# Load .env from repo root so agent/terminal doesn't need to source it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$SCRIPT_DIR/../.env" ]] && source "$SCRIPT_DIR/../.env"

case "$1" in
  get)
    host=""
    while IFS= read -r line; do
      [[ "$line" = host=* ]] && host="${line#host=}"
    done
    if [[ "$host" == "github.com" && -n "${GITHUB_TOKEN:-}" ]]; then
      echo "username=cebenestelli-miti"
      echo "password=$GITHUB_TOKEN"
    fi
    ;;
  store|erase)
    # No-op: we don't store or erase
    ;;
  *)
    exit 1
    ;;
esac
