#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/ros2_tim_projekt}"

echo "Cistim workspace: $WS"
rm -rf "$WS/build" "$WS/install" "$WS/log"
rm -rf "$WS/dataset"/session_* 2>/dev/null || true
find "$WS" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true
find "$WS" -type f \( -name "*.pyc" -o -name "*.pyo" \) -delete 2>/dev/null || true
find "$WS" -type f -name ".DS_Store" -delete 2>/dev/null || true

echo "Hotovo. Zostali iba zdrojaky, configy a skripty."
