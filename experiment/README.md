# SpiRob experiments

| Experiment | Purpose |
| --- | --- |
| [Exp 1 · Single-tendon cycling](#exp-1-bending-collection) | Measure hysteresis and cycle-to-cycle shakedown under repeated loading and unloading. |
| [Exp 2 · Two-tendon cycling](#exp-2-two-tendon-bending-collection) | Measure how two tendons interact when their loads overlap and release in sequence. |
| [Exp 3 · Single-tendon creep and recovery](#exp-3-single-tendon-creep-and-recovery) | Fit an effective viscoelastic response for each motor and tendon channel at a fixed 10 s per leg. |

## Slack completion notifications

On Spirob, put `SLACK_BOT_TOKEN` and `SLACK_CHANNEL` in the repository-root
`.env`, which Git ignores. Use the channel ID watched by the Mac broker and
a bot with `chat:write` and access to that channel. The experiment CLIs post
automatically when Slack is configured. Install the updated `control_core` package on Spirob
and use the Python environment with `python-dotenv` installed.

On the Mac, arm the pager before launching the experiment over SSH:

```sh
broker pager --thread "$CODEX_THREAD_ID" --filter-string 'spirob-finished:'
```

The pager wakes on the next matching experiment completion.
The CLI prints the completion token and exit status, then posts them to Slack along with the
relative CSV path when available. Keep the SSH log to recover the result if
Slack delivery fails. Notification failures preserve the experiment exit code.

## Exp 1 bending collection

Run both collectors directly on Spirob. Use a Python environment with `control_core` and
Matplotlib installed, and source the ROS 2 workspace if your shell has not
already done so. Run `make install` from the repository root to set up the
shared environment. Both scripts use the installed runner and write pose and
force samples from RAM after motion stops. Keep the GPIO streamer and mocap publisher running with
rigid body names beginning with `arm_` on `/poses`. Both collectors take the
hardware lock directly. See the [Pi sensor service setup](../deploy/README.md)
for starting the streamer and bridge before a run.
From the repository root on Spirob:

```sh
.venv/bin/python -m experiment.exp1_bending_collection -N 0 -F 2.5 -C 3 \
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
.venv/bin/python -m experiment.exp2_bending_collection -F 2.5 -C 3 --name baseline \
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


## Exp 3 single-tendon creep and recovery

Run this directly on SpiRob with the same GPIO and mocap services. Exp 3
requires fresh `arm_base` and `arm_tip` poses. Start each repeat from the same
preparation and rest history. Keep the arm untouched during the recorded
recovery.

```sh
.venv/bin/python -m experiment.exp3_creep_recovery -N 0 -F 1.0 --name T0_rep1
```

Each run changes the target for one motor, selected by `-N 0` through `-N 5`.
The five other tendons retain a 0.10 kgf target. The active tendon starts and
finishes at 0.10 kgf. Both legs are fixed at 10 seconds, with 25 force
setpoint intervals of 0.40 seconds per leg. Use the same peak target across motors
for comparable nominal ramp slopes. At the default 1.0 kgf peak, the slope
magnitude is 0.09 kgf/s.

| Phase | Default duration |
| --- | ---: |
| Preload baseline | 30 s |
| Loading leg | 10 s |
| Peak hold | 180 s |
| Unloading leg | 10 s |
| Recovery at preload | 300 s |

The table holds the force at the midpoint of each ramp interval, and
commands the exact peak and preload at the respective leg endpoints.
The default run takes 530 s. The overall limit is 600 s. If omitted, the run name includes the tendon,
such as `exp3_<timestamp>_tendon_1`.

Fit the model separately for each channel using the measured tension history in
`controls.csv` and the base-relative motion from `poses.csv`. Keep
`actuation.csv` with the recording so the nominal forcing and measured
forcing can both be replayed. The fitted parameters describe the assembled
motor, tendon, and backbone response over the tested force range and
10 s/leg protocol. For later validation, freeze the fitted parameters and
predict a held-out sequence with staggered and overlapping tendon loads.
Exp 2's overlap pattern is a starting point for that sequence. Use the same
preload, force range, and 10 s ramp duration as in Exp 3.

### Exp 3 command sheet

Prepare the arm, start one GPIO/HX711 streamer and one mocap bridge, and
source the ROS workspace on SpiRob. Copy **one trial line at a time** from
[the Exp 3 command sheet](exp3_batch.sh). Each motor has three 530 s fitting
repeats. Calibrate or home between commands as needed.

Fitting repeats go under `output/experiment/exp3_fit/T0/rep_1/` and the corresponding
folders for the other motors and repeats. Each trial writes its own
timestamped CSV and raw bundle.

### Model-fitting CSVs from Exp 1

From the checkout that holds the raw recordings:

```sh
.venv/bin/python -m experiment.postprocess_hysteresis
```

The exporter reads local single-tendon Exp 1 bundles and writes one CSV per
complete cycle under `output_postprocessed/tendon_N/<trial>/<run_id>/`.
It keeps earlier complete cycles from interrupted trials, with the original
run status in every file and in `manifest.csv`. Cycles with a gap greater
than 0.25 s in the aligned data are excluded. A force sample must precede
the pose by at most 0.10 s.

Poses are relative to the matching `arm_base` pose in the same mocap frame.
Each row retains trial elapsed time, measured force, and the original cycle
number. The pose origin is shared across cycles so residual bending and
shakedown remain visible. Fit consecutive cycles with their loading history
when estimating a memory-dependent model.

Add `--no-poses` to any direct collector for a force-only recording. This
mode skips the mocap subscription and pose checks. The main CSV contains
the same force and motor telemetry as the raw `controls.csv`, and the
run bundle has no `poses.csv`. The GPIO/HX711 streamer is still required.
