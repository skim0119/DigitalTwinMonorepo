# SpiRob sensor services

Run one GPIO streamer and one ROS 2 mocap bridge as independent systemd
services. Start experiments from a terminal on the Pi or over SSH.

Before setup, stop any hand-started copies of the streamer or mocap bridge. If upgrading from the HTTP deployment, stop
and disable the old `spirob-experiment@$USER.service` while the rig is idle,
before installing these units:

```sh
sudo systemctl disable --now "spirob-experiment@$USER.service"
```

Remove the retired unit template and any instance override from
`/etc/systemd/system/` if they were installed there.

```sh
cd ~/spirob_py_code
make setup-pi
systemctl status "spirob-gpio@$USER.service" "spirob-mocap@$USER.service"
```

Run `make setup-pi` as the Pi account that owns the checkout. The target invokes
`sudo` for service installation. Once systemd owns both processes, remove any
lines in `~/.bashrc` that launch the streamer or bridge to avoid duplicate
instances. You can keep the ROS environment setup line for interactive trials.

The mocap unit expects `~/ros2_spirob_ws/install/setup.bash` and a configured
mocap source that publishes the arm's rigid bodies on `/poses`. The streamer
serves load-cell samples locally on UDP port 5050.

With the direct collector dependencies installed, start a trial in the
foreground on the Pi:

```sh
cd ~/spirob_py_code
make install
source ~/ros2_spirob_ws/install/setup.bash
.venv/bin/python -u -m experiment.exp1_bending_collection -N 0 -F 1.0 -C 1
```

See [the experiment commands](../experiment/README.md) for parameters and
output locations.

## Remove the sensor services

When the rig is idle, run `make remove-pi` as the same account used for setup.
It stops and disables that account's two services and uninstalls the shared
unit templates from the Pi. The built streamer, ROS workspace, and experiment
recordings stay on disk.
