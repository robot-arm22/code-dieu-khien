#!/usr/bin/env bash
cd "$(dirname "$0")"
. .venv/bin/activate
python - <<'PY'
print("Python OK")
import tkinter; print("tkinter OK")
import customtkinter; print("customtkinter OK")
import PIL; print("Pillow OK")
import serial; print("pyserial OK")
from vision.cv2_safe import get_cv2
cv2=get_cv2(); print("cv2 OK", cv2.__version__)
PY
