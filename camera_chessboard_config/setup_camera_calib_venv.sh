#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv_camera_calib"

python3 -m venv --system-site-packages "$VENV_DIR"
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip setuptools wheel

# Na Jetsonovi casto existuje cv2 zo systemu. Najprv ho otestuj.
if python - <<'PY'
import cv2
print('cv2 OK:', cv2.__version__)
PY
then
  python -m pip install PyYAML numpy
else
  echo "[WARN] cv2 nie je dostupne. Skusam pip install opencv-python..."
  python -m pip install -r "$SCRIPT_DIR/requirements.txt" || true
  if ! python - <<'PY'
import cv2
print('cv2 OK:', cv2.__version__)
PY
  then
    echo "[ERROR] OpenCV stale nejde. Na Jetson/Ubuntu pouzi:"
    echo "        sudo apt update && sudo apt install python3-opencv"
    exit 1
  fi
fi

python -m pip install PyYAML numpy

echo ""
echo "Hotovo. Aktivacia venv:"
echo "source $VENV_DIR/bin/activate"
