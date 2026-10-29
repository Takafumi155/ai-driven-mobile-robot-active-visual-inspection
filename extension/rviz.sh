#!/usr/bin/env bash
set -euo pipefail
xvfb-run -a -s '-screen 0 1280x800x24' python3 extension/rviz_check.py
