#!/usr/bin/env bash
# Run Exp 1 and inter-trial homing over HTTP.
set -euo pipefail

usage() {
    cat <<'HELP'
Usage: ./exp1_local.sh [--dry-run]

Run ten 10 kgf cycles at each leg duration from 5 through 13 seconds.
SPIROB_API_URL defaults to http://127.0.0.1:5052.
SPIROB_TENDON defaults to 0 (zero-based).
The Experiment API must support POST /home for live sweeps.
--dry-run saves and validates all tables without submitting trials or homing.
HELP
}

dry_flag=
case "${1-}" in
    "") ;;
    --dry-run) dry_flag=--dry-run ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac
if (( $# > 1 )); then
    usage >&2
    exit 2
fi

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export SPIROB_API_URL=${SPIROB_API_URL:-http://127.0.0.1:5052}
tendon=${SPIROB_TENDON:-0}
steps=50
api=${SPIROB_API_URL%/}

if [[ -z "$dry_flag" ]]; then
    curl -fsS --max-time 15 "$api/health" | python3 -c '
import json, sys
if not json.load(sys.stdin).get("homing_supported"):
    sys.exit("This Experiment API does not offer homing. Sweep stopped before motion.")
'
fi

run_trial() {
    local leg=$1 dwell
    # Fifty increments per leg gives 0.2 kgf steps.
    dwell=$(printf '0.%02d' "$((leg * 100 / steps))")
    printf 'Running Exp 1 at %s s/leg\n' "$leg"
    python3 "$project_dir/experiment/exp1_bending_through_api.py" \
        --api-url "$SPIROB_API_URL" -N "$tendon" -F 10 -C 10 \
        --steps "$steps" --dwell-s "$dwell" --name "exp1_leg_${leg}s" \
        ${dry_flag:+"$dry_flag"}
}

run_homing() {
    if [[ -n "$dry_flag" ]]; then
        return 0
    fi

    printf 'Homing before the next trial\n'
    if ! curl -fsS --max-time 180 -X POST "$api/home" | python3 -c '
import json, sys
result = json.load(sys.stdin)
print(json.dumps(result))
sys.exit(0 if result.get("homed") is True else 1)
'; then
        echo "Homing failed. Sweep stopped before the next trial." >&2
        return 1
    fi
}

run_trial 5
run_homing
run_trial 6
run_homing
run_trial 7
run_homing
run_trial 8
run_homing
run_trial 9
run_homing
run_trial 10
run_homing
run_trial 11
run_homing
run_trial 12
run_homing
run_trial 13
