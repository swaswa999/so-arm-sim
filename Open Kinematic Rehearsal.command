#!/bin/zsh
cd "$(dirname "$0")" || exit 1
if [[ ! -x .venv/bin/mjpython ]]; then
  print "Set up this repository's Python environment first; see README.md."
  read -r "?Press Return to close."
  exit 1
fi
exec .venv/bin/mjpython full_arm_preview.py
