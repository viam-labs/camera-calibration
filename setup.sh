#!/bin/bash
set -euo pipefail

OS="$(uname -s)"
if [[ "$OS" == "Linux" ]]; then
    if ! dpkg -l libnlopt-dev >/dev/null 2>&1; then
        sudo apt-get update || true
        sudo apt-get install -y --no-install-recommends \
            -o DPkg::Lock::Timeout=60 \
            ca-certificates \
            libnlopt-dev
    fi
elif [[ "$OS" == "Darwin" ]]; then
    brew tap viamrobotics/brews
    brew install nlopt-static
fi
