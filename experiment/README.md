## Slack completion notifications

On Spirob, put `SLACK_BOT_TOKEN` and `SLACK_CHANNEL` in the repository-root
`.env`, which Git ignores. Use the channel ID watched by the Mac broker and
a bot with `chat:write` and access to that channel. Both bending CLIs post
automatically when Slack is configured. Install the updated `control_core` package on Spirob
and use the Python environment with `python-dotenv` installed.

On the Mac, arm the pager before launching the experiment over SSH:

```sh
broker pager --thread "$CODEX_THREAD_ID" --filter-string 'spirob-finished:'
```

The pager wakes on the next matching completion from either bending CLI.
The CLI prints the completion token and exit status, then posts them to Slack along with the
relative CSV path when available. Keep the SSH log to recover the result if
Slack delivery fails. Notification failures preserve the experiment exit code.

# Exp 1 bending collection

Run both collectors directly on Spirob. Use a Python environment with `control_core` and
Matplotlib installed, and source the ROS 2 workspace if your shell has not
already done so. Keep the GPIO streamer and mocap publisher running with
rigid body names beginning with `arm_` on `/poses`. Both collectors take the
hardware lock directly. Stop the Experiment API service first if it is running
on Spirob.
From the repository root on Spirob:

```sh
python3 experiment/exp1_bending_collection.py -N 0 -F 2.5 -C 3 \
  --max-tightening-steps "$MAX_TIGHTENING_STEPS"
```

`-N` is zero-based, so tendon 0 is `tendon_1` in Motion Board. `-F` sets the
peak in kgf, and `-C` sets the number of cycles. The defaults are tendon 0,
2.5 kgf, and one cycle. The five other tendons hold a 0.1 kgf force setpoint
throughout the run. Only the selected tendon changes setpoint. Each cycle
steps that tendon from zero up to the peak and back
down in 0.2 kgf increments, holding each level for one second by default.
A shorter final increment reaches each endpoint exactly. Use `--step-kgf`
and `--dwell-s` to tune the staircase, or use `--steps` to set an equal
number of increments per phase. Set the
hold long enough for the measured load and pose to settle. The export records
the entire response through each hold.

Add `--single-mode` to run force PID only on the motor selected by `-N`.
The controller leaves the other five motors inactive and applies no
pretension to them. Single-mode recordings use the name
`exp1_single_mode_<current_time>.csv`.

The collector uses the limits in `pins.json` (4.5 kgf, 3000 steps/s, and
3000 total tightening pulses by default). Use `--max-tightening-steps` to
override the pulse limit and `--max-tension-kg` to lower the tension limit. Every commanded
force must stay below that limit. Use `--config` for another pin and PID map.
Both experiments record every rigid body whose name starts with `arm_` and
require at least one fresh matching pose throughout the run. Configure the mocap
publisher to use `arm_tip` for the tip, `arm_base` for the base, and
`arm_link<N>` for the Nth link. The recordings retain the published names.

Run `python3 debug/watch_posture.py` to watch the same prefix on `/poses`.
Pass `--name` to watch specific bodies.

Both collectors accept `--name NAME` to append `_NAME` to the run folder
and exported CSV basename. For example, `--name baseline` produces
`exp1_<current_time>_baseline/` and `exp1_<current_time>_baseline.csv`.
Use 1–80 ASCII letters, digits, hyphens, or underscores.

The script writes `output/experiment/exp1_<current_time>.csv` on Spirob.
Each row contains one raw pose with its ROS timestamp,
frame, cycle-relative elapsed time, and the latest available command and
measured load for the selected tendon. Multiple bodies may share a ROS timestamp.
The terminal shows the current cycle, target force, and measured load about
once per second. Ctrl-C stops the trial and exports the samples already recorded. The sibling
`exp1_<current_time>/` directory contains the raw run files and
`diagnostics.png` for the full time series, plus `diagnostics_step_end.png`
with the last recorded pose and force samples within each completed hold.
`diagnostics_hysteresis.png` plots hold-end posture against measured force
for each tendon with a varying setpoint, with loading and unloading markers.
Download the CSV or the raw run directory from Spirob when you are ready
to review it.

In Motion Board, choose **Import bending CSV** in the Runs sidebar. The file
opens locally with one selectable trial per cycle, the recorded body
trajectories, a timeline, and the selected tendon's force traces.

## Exp 2 two-tendon bending collection

Run Exp 2 on Spirob with the same hardware setup and verified tightening pulse
limit as Exp 1.

```sh
python3 experiment/exp2_bending_collection.py -F 2.5 -C 3 --name baseline \
  --max-tightening-steps "$MAX_TIGHTENING_STEPS"
```

Exp 2 starts T0 and T1 at zero force and follows this cycle:

1. Raise T0 to the peak while T1 stays at zero.
2. Raise T1 to the peak while T0 holds its peak.
3. Lower T0 to zero while T1 holds its peak.
4. Lower T1 to zero.

The other four tendons hold 0.1 kgf. The peak applies to each active tendon
and defaults to 2.5 kgf. Both tendons hold peak force together at the end of
phase 2. The cycle count, step size, and dwell use the same defaults as Exp 1.

The collector saves `output/experiment/exp2_<current_time>.csv` with every
matching mocap body and both active tendon force channels. The CSV has one row per
pose and active tendon. Open it with **Import bending CSV** and use Motion
Board's Tendon selector to switch between T0 (`tendon_1`) and T1 (`tendon_2`).
With `--name baseline`, the CSV ends in `_baseline.csv` and the sibling
`exp2_<current_time>_baseline/` folder contains the raw recording. For both
experiments, the raw `controls.csv` records all six tendon channels, including
commanded force, measured load, motor position, and output step rate.

Both experiments generate their force tables with
[`force_staircase`](quasi_static.py). The trial runner holds each table
row until the next timestamp. The dwell is time-based, so choose it based on
the measured settling time.

## Run bending experiments from a local client through Experiment API

With the Pi's Experiment API reachable over HTTP, run this from the local checkout:

```sh
python3 experiment/exp1_bending_through_api.py --api-url http://127.0.0.1:5052 \
  -N 0 -F 2.5 -C 3 --step-kgf 0.2 --dwell-s 1

python3 experiment/exp2_bending_through_api.py --api-url http://127.0.0.1:5052 \
  -F 2.5 -C 3 --step-kgf 0.2 --dwell-s 1
```

Set `--api-url` to your forwarded or private API address. Add `--dry-run` to
save the actuation CSV locally and validate it against the API before
submitting a trial. A normal invocation starts a trial and confirms the
mechanical reference to the API, so check the live zero and preload before
running it. The local client needs only Python's standard library. It generates
the same force staircase as its corresponding collector, with all cycles in a
single actuation CSV and inactive tendons at 0.1 kgf. The server applies its
configured limits and holds each row until the next timestamp.

The client prints the trial ID, waits for terminal status, and saves the input
table and exported run files under `output/experiment/exp1_api_*/` or `output/experiment/exp2_api_*/`. Failed
or stopped trials also download the files available on the server.
Interrupting the client ends local waiting. Use the API's stop endpoint to
stop an active trial.

To add another API-driven experiment, define its arguments and actuation table
in its own script, then call `run_trial` from `experiment_client.py` with the
table bytes, duration, mode, and unit.
