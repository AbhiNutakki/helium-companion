#!/bin/zsh
set -eu
cd "$(dirname "$0")/.."
export PATH="/opt/homebrew/bin:$PATH"
if ! command -v gz >/dev/null; then print 'Install Gazebo Jetty first.'; exit 1; fi
if ! test -x .venv/bin/python; then /opt/homebrew/bin/python3.14 -m venv --system-site-packages .venv; fi
.venv/bin/python -m pip install -r requirements.txt
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build -j 4
