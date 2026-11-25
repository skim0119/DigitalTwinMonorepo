# Debug tools

From the repository root, install the debug scripts' dependencies in the shared `.venv`:

```sh
make install
source .venv/bin/activate
```

The PID tuner also needs a Raspberry Pi with the HX711 streamer running.

## Xbox controller

On the Pi with an Xbox controller connected, run this from the repo root:

```sh
python3 debug/xbox_controller_6motors.py
```

This uses Pygame for input and the native controller for STEP/DIR output. To identify the controller's
button indices without driving the motors, append `--list-buttons`.

## PID tuning and single-motor jog

[pins.json](../pins.json) contains the BCM pin map, each motor's PID gains,
and the shared force deadband in grams. The PID controller and tuner use
these gains by default. The tuner's `--kp`, `--ki`, `--kd`, and `--deadband-g`
options override them for one trial.

With the Pi's HX711 streamer running, install the current native controller
in the active Python environment:

```sh
make install
```

**One-shot tuning trial.** Run a timed trial with the configured gains, or
supply overrides such as `--kp`. Set the target and verified limits for your
motor before running:

```sh
python3 debug/PID_tune_motor.py --motor 0 --target-kg "$TARGET_KG" --seconds 3 \
  --max-tension-kg "$MAX_TENSION_KG" --max-tightening-steps "$MAX_STEPS"
```

PID Multi and the one-shot tuner save timestamped CSVs, a force plot, and
`run.json` with the effective settings and limits under root `output/`.
The tuner tests the supplied gains and prints metrics and a copy-ready `"pid"`
entry. Gain selection and saving the entry in `pins.json` are manual. The run
starts with a 0.1 kg baseline hold before moving to the requested targets.
If the final test target is above 0.1 kg, the tuner returns to 0.1 kg for
one more hold before stopping. All holds use `--seconds`. Safety stops and
interruptions end the run immediately.

**Target-switch validation.** Repeat `--target-kg` to test a sequence with the
same gains in a single session. `--seconds` is the hold time for each target.
The report gives tracking RMSE, settling time, and overshoot or undershoot for
each hold.

```sh
python3 debug/PID_tune_motor.py --motor 0 \
  --target-kg "$LOW_KG" --target-kg "$MID_KG" --target-kg "$HIGH_KG" --target-kg "$LOW_KG" \
  --seconds 5 --max-tension-kg "$MAX_TENSION_KG" --max-tightening-steps "$MAX_STEPS"
```

**Live hand tuning.** Run from a desktop session on the Pi, or use X
forwarding with a graphical Matplotlib backend:

```sh
python3 debug/PID_live_tune.py --motor 0 --target-kg "$TARGET_KG" \
  --max-tension-kg "$MAX_TENSION_KG" --max-tightening-steps "$MAX_STEPS"
```

The plot shows the last 15 seconds of measured load and target. Press
**Start**, then edit the target or gains and click **Apply settings**.
Press **Stop** to end motion. Use `--help` on either script for all options.

Try a simulated 25-pulse jog before using hardware:

```sh
python3 debug/simple_cmdline_controller.py 0 forward --steps 25 --dry-run
```

The jog prints the motor number, BCM pins, requested speed, and pulse count.
It defaults to 100 pulses at 3 RPM. Remove `--dry-run` to send those pulses
to the selected motor on the Pi.

## Xbox recording

With the HX711 streamer and ROS 2 `/poses` publisher running on the Pi (and the
ROS workspace sourced), record while using the same Xbox controls:

```sh
python3 debug/xbox_controller_recording.py --output output/xbox_recording.csv
```

The output must be a new file in an existing directory. Press Ctrl+C to stop.
The CSV is flushed about every 0.1 seconds and has one row per `arm_*` rigid
body per sample; before any pose arrives, a row has empty pose fields. Rows
contain UTC and elapsed timestamps, the source ROS pose timestamp and age,
six HX711 loads in kgf and their ages, and six motor step positions. Loads
older than one second are blank; pose age indicates when a pose is stale.
Motor indices are zero-based. The readings are latest available snapshots,
not hardware-synchronized measurements. Manual Xbox control does not enforce
load-cell force limits. On Linux, the terminal reports the recorder process's
current resident memory (RSS) every five seconds.
