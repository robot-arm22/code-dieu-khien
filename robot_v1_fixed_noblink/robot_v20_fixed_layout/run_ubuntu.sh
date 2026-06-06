#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ~/venv/bin/activate

export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export OPENCV_VIDEOIO_PRIORITY_MSMF=0
export PYTHONFAULTHANDLER=1

python -X faulthandler main.py
