#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

sudo apt update
sudo apt install -y python3 python3-pip python3-venv python3-tk libzbar0 v4l-utils libgl1 libglib2.0-0 libxrender1 libxext6 libsm6

rm -rf .venv
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade 'pip<26'
pip uninstall -y opencv-python opencv-contrib-python opencv-python-headless numpy || true
pip install --no-cache-dir -r requirements_ubuntu.txt

echo ""
echo "Cài xong. Chạy phần mềm bằng:"
echo "  ./run_ubuntu.sh"
echo ""
echo "Nếu không mở được cổng USB/Arduino/ESP32, chạy thêm rồi đăng xuất/đăng nhập lại:"
echo "  sudo usermod -aG dialout $USER"
