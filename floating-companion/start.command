#!/bin/zsh
set -eu
cd "$(dirname "$0")"
export PATH="/opt/homebrew/bin:$PATH"
ulimit -n 10240 2>/dev/null || true
if ! test -x .venv/bin/python || ! test -f build/libCompanion.dylib; then ./scripts/setup.sh; fi
exec .venv/bin/python app.py --open "$@"
