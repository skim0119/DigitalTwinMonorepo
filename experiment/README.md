# Exp 1 bending collection

Run Exp 1 directly on Spirob. Use a Python environment with `control_core` and
Matplotlib installed, and source the ROS 2 workspace if your shell has not
already done so. Keep the GPIO streamer and mocap publisher running with
rigid body names beginning with `arm_` on `/poses`. If the Experiment API service is running
on Spirob, stop it separately so the collector can take the hardware lock.
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

Set `MAX_TIGHTENING_STEPS` to the verified pulse limit for your setup. The
collector uses the global limits in `pins.json` (4.5 kgf and 3000 steps/s by
default). Use `--max-tension-kg` to lower the tension limit. Every commanded
force must stay below that limit. Use `--config` for another pin and PID map.
By default, both experiments record every rigid body whose name starts with
`arm_` and require at least one fresh matching pose throughout the run.
For Exp 1, use `--name arm_tip` to select only the tip, or repeat `--name`
to require several exact names. With exact names, the collector stops if any
selected body goes stale.

Run `python3 debug/watch_posture.py` to watch the same prefix on `/poses`.
Pass `--name` to watch specific bodies.

The script writes `output/experiment/exp1_<current_time>.csv` on Spirob.
Each row contains one raw pose with its ROS timestamp,
frame, cycle-relative elapsed time, and the latest available command and
measured load for the selected tendon. The two bodies may share a ROS timestamp.
Ctrl-C stops the trial and exports the samples already recorded. The sibling
`exp1_<current_time>/` directory contains the raw run files and
`diagnostics.png`. Download the CSV or the raw run directory from Spirob when
you are ready to review it.

In Motion Board, choose **Import bending CSV** in the Runs sidebar. The file
opens locally with one selectable trial per cycle, the recorded body
trajectories, a timeline, and the selected tendon's force traces.

## Exp 2 two-tendon bending collection

Exp 2 uses [Experiment API](../experiment-api/README.md). The default API
address is `http://127.0.0.1:5052`. Use `--api-url` to select another address.

```sh
python3 experiment/exp2_bending_collection.py -F 2.5 -C 3
```

Exp 2 starts T0 and T1 at zero force and follows this cycle:

1. Raise T0 to the peak while T1 stays at zero.
2. Raise T1 to the peak while T0 holds its peak.
3. Lower T0 to zero while T1 holds its peak.
4. Lower T1 to zero.

The other four tendons hold 0.1 kgf. The peak applies to each active tendon
and defaults to 2.5 kgf. Both tendons hold peak force together at the end of
phase 2. The cycle count, step size, and dwell use the same defaults as Exp 1.

The collector saves `output/experiment/exp2_<current_time>.csv` with both
mocap bodies and both active tendon force channels. The CSV has one row per
pose and active tendon. Open it with **Import bending CSV** and use Motion
Board's Tendon selector to switch between T0 (`tendon_1`) and T1 (`tendon_2`).

Both experiments generate their force tables with
[`force_staircase`](quasi_static.py). The trial runner holds each table
row until the next timestamp. The dwell is time-based, so choose it based on
the measured settling time.
