#!/bin/zsh
cd "$(dirname "$0")" || exit 1
exec .venv/bin/mjpython workcell_preview.py
