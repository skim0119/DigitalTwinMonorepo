# Spirob

Tools for controlling the Spirob arm and monitoring its Raspberry Pi.

| Tool | Purpose |
| --- | --- |
| [`debug/`](debug/) | Standalone hardware debugging scripts, including Xbox jogging. |
| [`control_core/`](control_core/README.md) | Installable C++ motor control library used by the experiment API. |
| [`experiment-api/`](experiment-api/README.md) | Runs trials on the Pi and saves recorded poses, controls, and optional webcam video. |
| [`motion-board/`](motion-board/README.md) | Local browser console for uploading actuation CSVs, reviewing trajectories, and downloading results. |
| [`stream-gpio/`](stream-gpio/README.md) | C++ service on the Pi that streams GPIO levels and can read HX711 load cells for monitoring and force control. |
| [`stream-gpio-monitor/`](stream-gpio-monitor/README.md) | Rust monitor that connects to the Pi over SSH and serves a browser dashboard. |
| [`push`](push) and [`pull`](pull) | Sync locally present tracked files with the Pi. Push also sends new experiment and deployment sources. Both support `--dry-run`. Rebuild the GPIO streamer on the Pi after pushing source changes. |

The Pi service reads the HX711s. Live experiments use native PID and motor
workers through `control_core`. The monitor displays telemetry from the
WebSocket stream.

## Install and launch experiments

### Local simulation

From the repository root, start Experiment API with Python 3.11 or newer:

```sh
python3 -m venv .experiment-venv
source .experiment-venv/bin/activate
python -m pip install ./experiment-api
experiment-api --simulate --data /tmp/spirob-demo
```

In a second terminal, start Motion Board with Node.js 22.12 or newer:

```sh
cd motion-board
npm ci
npm run build
npm start
```

Open [Motion Board](http://127.0.0.1:8082).

### Raspberry Pi hardware

On Spirob, from the repository root, start the [GPIO streamer](stream-gpio/README.md)
and mocap bridge so the Pi has live HX711 readings and ROS 2 `/poses`.
Install and launch Experiment API with the arm's measured calibration:

```sh
source ~/ros2_spirob_ws/install/setup.bash
python3 -m venv --system-site-packages .experiment-venv
source .experiment-venv/bin/activate
python -m pip install ./control_core ./experiment-api
experiment-api --calibration /path/to/verified-calibration.json --data ~/spirob-experiments
```

On your computer, install and build Motion Board as above, then run
`npm start -- --ssh spirob` from `motion-board/`. This uses your existing
OpenSSH host alias. See [Experiment API setup](experiment-api/README.md)
for the calibration format, hardware prerequisites, and optional webcam setup.

## Debug tools

Install the debug scripts' Python dependencies in your active environment:

```sh
python3 -m pip install ./control_core -r debug/requirements.txt
```

The PID tuner also needs a Raspberry Pi with the HX711 streamer running.

### Xbox controller

On the Pi with an Xbox controller connected, run this from the repo root:

```sh
python3 debug/xbox_controller_6motors.py
```

This uses Pygame for input and the native controller for STEP/DIR output. To identify the controller's
button indices without driving the motors, append `--list-buttons`.

### PID tuning and single-motor jog

[pins.json](pins.json) contains the BCM pin map, each motor's PID gains,
and the shared force deadband in grams. The PID controller and tuner use
these gains by default. The tuner's `--kp`, `--ki`, `--kd`, and `--deadband-g`
options override them for one trial.

With the Pi's HX711 streamer running, install the current native controller
in the active Python environment:

```sh
python3 -m pip install ./control_core
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
