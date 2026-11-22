#!/usr/bin/env bash
set -euo pipefail

# Run from the Spirob checkout after restoring wiring and verifying the limits.
# Preview with bash exp1_nonlinear_check.sh. Add --run to enable collection.
# MAX_TIGHTENING_STEPS overrides the pulse limit in pins.json.
# MAX_TENSION_KG optionally lowers the cutoff from the configured default.
# For the batch notification, set the broker filter to spirob-batch-finished:
notify_batch() {
    local status=$1
    local message="spirob-batch-finished: exp1_nonlinear_check exit=$status"
    printf '%s\n' "$message"
    if ! .venv/bin/python - "$message" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

from control_core.slack import load_settings

try:
    settings = load_settings(Path(".env"))
    if settings is not None:
        token, channel = settings
        response = subprocess.run(
            ["curl", "--silent", "--show-error", "--fail", "--max-time", "10",
             "--header", "@-", "--header", "Content-Type: application/json",
             "--data-binary", json.dumps({"channel": channel, "text": sys.argv[1]}),
             "https://slack.com/api/chat.postMessage"],
            input=f"Authorization: Bearer {token}\n",
            text=True, stdout=subprocess.PIPE, check=True,
        )
        if json.loads(response.stdout).get("ok") is not True:
            raise ValueError("Slack rejected the batch message")
except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError):
    sys.exit(1)
PY
    then
        echo "Slack delivery of the batch status could not be confirmed." >&2
    fi
}

case "${1:-}" in
    '') ;;
    --run) ;;
    *)
        echo "Usage: bash exp1_nonlinear_check.sh [--run]" >&2
        exit 2
        ;;
esac

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ "${1:-}" == --run ]]; then
    trap 'notify_batch "$?"' EXIT
    # ROS setup hooks may reference unset variables.
    set +u
    source "$HOME/ros2_spirob_ws/install/setup.bash"
    set -u
    export MPLBACKEND=Agg
fi

common=(.venv/bin/python -u experiment/exp1_bending_collection.py -N 4 -F 2.5 -C 6)
if [[ -n "${MAX_TIGHTENING_STEPS:-}" ]]; then
    common+=(--max-tightening-steps "$MAX_TIGHTENING_STEPS")
fi
if [[ -n "${MAX_TENSION_KG:-}" ]]; then
    common+=(--max-tension-kg "$MAX_TENSION_KG")
fi

# Dwell sweep at 13 equal increments per leg. The step-count sweep
# uses 7 and 26 increments with dwell adjusted to keep each leg at 13 s.
# The original 0.2 kgf staircase brackets the sweep to reveal drift.
while read -r name dwell increment_flag increment; do
    command=("${common[@]}" --name "nonlinear_$name"
        --dwell-s "$dwell" "$increment_flag" "$increment")
    printf '%q ' "${command[@]}"
    printf '\n'
    if [[ "${1:-}" == --run ]]; then
        "${command[@]}"
    fi
done <<'BLOCKS'
baseline_start 1 --step-kgf 0.2
steps13_mid 1 --steps 13
steps13_fast 0.5 --steps 13
steps13_slow 2 --steps 13
steps7_mid 1.857142857142857 --steps 7
steps26_mid 0.5 --steps 26
baseline_end 1 --step-kgf 0.2
BLOCKS
