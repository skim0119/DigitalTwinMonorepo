# Experiment API

Experiment API validates actuation tables, runs one batch at a time, records
each trial locally, and exposes live previews and CSV downloads. ROS 2 stays on
the Pi. Motion Board and other clients use HTTP and WebSocket over SSH.

## Try the workflow without hardware

From the repository root, with Python 3.11 or newer:

```sh
python3 -m venv .experiment-venv
source .experiment-venv/bin/activate
pip install ./experiment-api
experiment-api --simulate --data /tmp/spirob-demo
```

Open [Motion Board](../motion-board/README.md) in a second terminal. The API
listens on `127.0.0.1:5052`. Simulation generates synthetic poses and loads
for both control modes.

## Use Spirob

On the Pi, source the ROS 2 workspace that contains the mocap package and install
the API and native control package. A system-site-packages virtual environment
lets the Jazzy Python interpreter import its system dependencies.

```sh
source ~/ros2_spirob_ws/install/setup.bash
python3 -m venv --system-site-packages .experiment-venv
source .experiment-venv/bin/activate
pip install ./control_core ./experiment-api
experiment-api --calibration /path/to/verified-calibration.json --data ~/spirob-experiments
```

Run from the repository root or set `SPIROB_PINS_CONFIG` to the absolute path to
`pins.json`. Start the existing mocap bridge with a live `/poses` stream and
the updated GPIO streamer. The native controller uses the
streamer's `hx711-fast` UDP endpoint. Stop other motor controllers and
load-cell readers before operating. Match `SPIROB_MONITOR_PORT` to the
streamer's port if it differs from 5050.

A calibration JSON must have six signed, nonzero `steps_per_mm` values, six
`travel_mm` pairs containing zero, a positive `max_force_kgf`, and a positive
`max_step_rate` no greater than the controller's 2133 steps/s limit. The sign
maps positive CSV travel to the calibrated motor direction. Supply measured
values for the installed setup. The Pi uses commanded steps to estimate travel.
It has no independent absolute-position measurement.

Before enabling the drivers, establish the mechanical zero and preload, and
verify step direction, travel, and stop behavior on the Pi. The browser requires
confirmation of the reference before a hardware batch starts. If a trial leaves
any motor away from step zero, later trials in the batch cannot run. Re-establish
zero and restart the service before submitting another batch. Stop cancels the
remaining trials and preserves the partial record. WebSocket disconnects do
not interrupt an accepted trial. The local controller stops on stale feedback
or an expired command.

## CSV contract

```csv
time_s,tendon_1,tendon_2,tendon_3,tendon_4,tendon_5,tendon_6
0,0,0,0,0,0,0
1,2,0,0,0,0,0
2,2,0,0,0,0,0
3,0,0,0,0,0,0
4,0,0,0,0,0,0
```

Choose units explicitly. **Force** accepts N or kgf and holds each setpoint
until the following row. **Displacement** accepts mm relative to the trial
reference and linearly interpolates targets between rows. Live displacement
tables start and end at zero. Include time at zero before the final timestamp
if the motors need to settle back to the reference. The final timestamp ends
playback. These semantics can be compared with the simulator when available.

## API and run files

- `POST /validate?mode=force&unit=N` checks and previews a CSV request body.
- `POST /runs?mode=force&unit=N&repeats=3&reference_ready=true` submits the CSV
  body and returns one ID per trial. Uploading to `/validate` never starts a trial.
- `GET /runs` lists trials. `POST /runs/{id}/stop` cancels the active batch.
- `GET /runs/{id}/files/{name}?download=true` downloads the stored file.
- `GET /runs/{id}/samples?kind=poses` returns pose rows. Use `kind=controls` for control rows.
- `GET` and `PUT /runs/{id}/review` read and save reversible review annotations.
- `WS /events` carries run status, with batched pose and control previews. It
  may omit samples when the viewer falls behind. Completed run files still
  contain the recorded samples.

Each run directory contains `actuation.csv`, `run.json`, `poses.csv`,
`controls.csv`, and `review.json` when annotations have been saved. Use
`elapsed_s` to align streams and trials. Pose rows retain the ROS message
timestamp, frame, rigid-body name, position in meters, and quaternion in XYZW
order. The control rows are sampled at the runner update rate and contain the
applied setpoint, observed filtered load, motor step count, and step rate.
Pose and control rows have independent timestamps. Partial recordings and
failure reasons are preserved.

## Tests

```sh
pip install '.[test]'
python -m unittest experiment_api.test_experiment -v
```
