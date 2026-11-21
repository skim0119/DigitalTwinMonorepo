# Exp 1 bending collection

Update and start [Experiment API](../experiment-api/README.md) on the board with
the verified hardware calibration, GPIO streamer, and live ROS 2 `/poses`
containing `arm_base` and `arm_tip`. From the repository root on the board:

```sh
python3 experiment/exp1_bending_collection.py -N 0 -F 2.5 -C 3 --reference-ready
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

The API enforces the calibrated force limit, which must exceed the requested
peak. It checks for recent poses from both bodies and stops recording if either
stream goes stale. The cycles share one calibrated reference. The selected
tendon returns to a zero force setpoint between cycles. A zero force setpoint does not establish a
mechanical zero. To use a simulator or an SSH tunnel from another machine, pass
`--api-url http://127.0.0.1:PORT`. For simulation, omit `--reference-ready`.

The script writes `output/experiment/exp1_<current_time>.csv` on the machine
running the script. Each row contains one raw pose with its ROS timestamp,
frame, cycle-relative elapsed time, and the latest available command and
measured load for the selected tendon. The two bodies may share a ROS timestamp.
Ctrl-C requests a batch stop and exports the samples already recorded. The
original run files also remain in the API's recording directory.

In Motion Board, choose **Import bending CSV** in the Runs sidebar. The file
opens locally with one selectable trial per cycle, the base and tip
trajectories, a timeline, and the selected tendon's force traces.

## Exp 2 two-tendon bending collection

```sh
python3 experiment/exp2_bending_collection.py -F 2.5 -C 3 --reference-ready
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
[`force_staircase`](quasi_static.py). The Experiment API holds each table
row until the next timestamp. The dwell is time-based, so choose it based on
the measured settling time.
