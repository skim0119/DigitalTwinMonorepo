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
