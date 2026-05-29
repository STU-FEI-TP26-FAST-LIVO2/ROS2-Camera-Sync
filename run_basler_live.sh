#!/usr/bin/env bash
set -eo pipefail

PYLON_VIEWER="/opt/pylon/bin/pylonviewer"

echo "=== Basler live preview cez pylonviewer ==="

if [ ! -x "${PYLON_VIEWER}" ]; then
  echo "Chyba: nenasiel som ${PYLON_VIEWER}"
  echo "Skontroluj instalaciu pylon SDK."
  exit 1
fi

exec "${PYLON_VIEWER}"
