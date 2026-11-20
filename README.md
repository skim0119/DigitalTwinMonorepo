# Spirob

Tools for controlling the Spirob arm and monitoring its Raspberry Pi.

| Tool | Purpose |
| --- | --- |
| [`Spirob_Code/`](Spirob_Code/README.md) | Python arm control, including motor drivers, force PID, and experiment scripts. |
| [`debug/`](debug/) | Standalone hardware debugging scripts, including Xbox jogging. |
| [`control_core/`](control_core/README.md) | Installable C++ motor control library used by the experiment API. |
| [`stream-gpio/`](stream-gpio/README.md) | C++ service on the Pi that streams GPIO levels and can read HX711 load cells for monitoring and force control. |
| [`stream-gpio-monitor/`](stream-gpio-monitor/README.md) | Rust monitor that connects to the Pi over SSH and serves a browser dashboard. |
| [`push`](push) and [`pull`](pull) | Sync tracked files with the Pi. Both support `--dry-run`. |

The Pi service reads the HX711s. Live experiments use native PID and motor
workers through `control_core`. The monitor displays telemetry from the
WebSocket stream.

## Debug tools

Install the debug scripts' Python dependencies in your active environment:

```sh
python3 -m pip install -r debug/requirements.txt
```

The PID tuner also needs a Raspberry Pi with the HX711 streamer running.

### Xbox controller

On the Pi with an Xbox controller connected, run this from the repo root:

```sh
python3 debug/xbox_controller_6motors.py
```

This uses Pygame and the shared GPIO motor driver. To identify the controller's
button indices without driving the motors, append `--list-buttons`.

### PID tuning and single-motor jog

[pins.json](pins.json) contains the BCM pin map, each motor's PID gains,
and the shared force deadband in grams. The PID controller and tuner use
these gains by default. The tuner's `--kp`, `--ki`, `--kd`, and `--deadband-g`
options override them for one trial.

With the Pi's HX711 streamer running, use
`python3 debug/PID_tune_motor.py --help` to see the trial options.

PID Multi and the tuner write one folder per run under root `output/`. Each
folder has a UTC timestamp, per-motor CSV logs, a force plot, and `run.json`
with the effective PID settings and trial limits.

Try a simulated 25-pulse jog before using hardware:

```sh
python3 debug/simple_cmdline_controller.py 0 forward --steps 25 --dry-run
```

The jog prints the motor number, BCM pins, requested speed, and pulse count.
It defaults to 100 pulses at 3 RPM. Remove `--dry-run` to send those pulses
to the selected motor on the Pi.
