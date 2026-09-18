#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WEB_PORT="${WEB_PORT:-8080}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: python3 is required to serve the web UI." >&2
  exit 1
fi

if [[ ! -f "$SCRIPT_DIR/web/index.html" ]]; then
  echo "Error: web/index.html was not found." >&2
  exit 1
fi

echo "RosRobot Web UI: http://$(hostname -I | awk '{print $1}'):$WEB_PORT/index.html"
echo "Press Ctrl-C to stop the web server."
exec python3 -m http.server "$WEB_PORT" --bind 0.0.0.0 --directory "$SCRIPT_DIR/web"
