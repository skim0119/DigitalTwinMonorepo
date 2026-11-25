# SpiRob experiment runner

The direct experiment scripts use this Python library to play actuation
tables, record load-cell and ROS 2 pose data, and generate diagnostics after
motion stops. Run the collectors from the Pi checkout or through an SSH shell.
See [Pi sensor services](../deploy/README.md) for setup and
[the experiment commands](../experiment/README.md) for trial parameters.

Run `make install` from the repository root to install the runner in editable
mode alongside `control_core`, the experiment package, and the debug
dependencies. The Pi's ROS 2 workspace still needs to be sourced before
collection.

Each run stores `actuation.csv`, `run.json`, `poses.csv`, and `controls.csv`.
The runner buffers samples during motion and saves them after the controller
stops, including on handled faults. A process crash or power loss can lose
buffered samples. The metadata records the outcome and any error. Pose
positions are in meters and rotations use XYZW quaternion order. Use
`elapsed_s` to align the independently sampled pose and control streams.

Run the library tests with:

```sh
.venv/bin/python -m unittest experiment_runner.test_experiment
```

After `make install`, source the Pi ROS workspace and run
`.venv/bin/experiment-runner-server` from the checkout containing pins.json.
The service reserves the controller until it exits. It listens on port 8000
on all interfaces. Use it on a trusted LAN: any client with access can start motion.

Upload a table and save its result with:

    curl --fail --data-binary @spirob_force_actuation.csv 'http://spirob.local:8000/run?unit=N' -o result.csv

For this service, one input unit commands one kgf, including when labeled N.
The normal controller tension limit applies. Each result row contains an arm
body pose and the latest preceding target, measured load and sample time for
each input tendon. Empty fields mean no preceding control sample.
Runs are saved under server-runs. To change wiring, copy the packaged
experiment_runner/config.ini and pass its path with --config. The six motor
IDs correspond to tendon_1 through tendon_6 in upload order.
