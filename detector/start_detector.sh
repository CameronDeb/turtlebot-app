#!/usr/bin/env bash
# Start the object detector (detector/serve.py) in WSL. Usage: bash start_detector.sh [conda env, default detect]
set -e
ENV_NAME="${1:-detect}"
for base in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3"; do
  if [ -f "$base/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1091
    source "$base/etc/profile.d/conda.sh"
    break
  fi
done
conda activate "$ENV_NAME"
cd "$(dirname "$0")"
exec python -u serve.py "${@:2}"
