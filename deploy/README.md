# SpiRob Pi services

Systemd runs the GPIO streamer, ROS 2 mocap bridge, and experiment HTTP server.
Use the Pi account that owns `~/spirob_py_code`. The mocap and experiment units
use `~/ros2_spirob_ws/install/setup.bash`, and the mocap source must be
configured to publish the arm's rigid bodies on `/poses`.

While the rig is idle, back up `~/.bashrc`, remove the blocks that launch the
GPIO streamer and mocap publisher, and stop the copies those blocks started.
Keep the ROS setup and `poses` alias if you use them interactively.
If upgrading an older HTTP deployment, review any systemd overrides for
`spirob-experiment@$USER.service` before setup. Existing overrides still apply.

```sh
cd ~/spirob_py_code
make setup-pi
systemctl status "spirob-gpio@$USER.service" "spirob-mocap@$USER.service" "spirob-experiment@$USER.service"
```

Setup installs the Python packages into `.venv`, builds the streamer, and
uses `sudo` to install and enable all three units. Run setup again while the
rig is idle to reinstall the units and restart the services.

The GPIO streamer serves load-cell samples locally on UDP port 5050. The
experiment server listens on the Pi's loopback address on TCP port 8000.
From your workstation, open a tunnel and use `http://127.0.0.1:8000/run` in
your client:

```sh
ssh -N -L 8000:127.0.0.1:8000 spirob
```

For a foreground trial, stop the experiment server first so the collector can
acquire the hardware lock. Start the service again after the trial.

```sh
cd ~/spirob_py_code
sudo systemctl stop "spirob-experiment@$USER.service"
source ~/ros2_spirob_ws/install/setup.bash
.venv/bin/python -u -m experiment.exp1_bending_collection -N 0 -F 1.0 -C 1
sudo systemctl start "spirob-experiment@$USER.service"
```

See [the experiment commands](../experiment/README.md) for parameters and
output locations.

## Remove the Pi services

When the rig is idle, run `make remove-pi` as the same account used for setup.
It stops and disables the three services and removes their unit templates.
The built streamer, Python environment, ROS workspace, and experiment
recordings stay on disk.
