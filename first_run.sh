#!/bin/bash
# Runs once when the module is installed on a target machine. Installs
# system deps (nlopt for armplanning), uv, creates the Python venv,
# installs Python dependencies.

set -euo pipefail

cd "$(dirname "$0")"

bash ./setup.sh

if ! command -v uv >/dev/null && [ ! -x "$HOME/.local/bin/uv" ]; then
    curl -LsSf https://astral.sh/uv/install.sh | sh
fi

UV="$(command -v uv || echo "$HOME/.local/bin/uv")"

"$UV" venv --python=3.10 .venv --clear
"$UV" pip install --python .venv/bin/python -r requirements.txt
