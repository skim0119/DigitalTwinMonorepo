# Spirob

Tools for controlling the Spirob arm and monitoring its Raspberry Pi.

| Tool | Purpose |
| --- | --- |
| [`Spirob_Code/`](Spirob_Code/README.md) | Python arm control, including motor drivers, force PID, and experiment scripts. |
| [`debug/`](debug/) | Standalone hardware debugging scripts, including Xbox jogging. |
| [`stream-gpio/`](stream-gpio/README.md) | C++ service on the Pi that streams GPIO levels and can read HX711 load cells for monitoring and force control. |
| [`stream-gpio-monitor/`](stream-gpio-monitor/README.md) | Rust monitor that connects to the Pi over SSH and serves a browser dashboard. |
| [`push`](push) and [`pull`](pull) | Sync tracked files with the Pi. Both support `--dry-run`. |

In the force-control setup, the Pi service reads the HX711s, Python PID control
fetches readings over local UDP, and the monitor displays telemetry from the
WebSocket stream.

## Xbox controller debug tool

On the Pi with an Xbox controller connected, run this from the repo root:

```sh
python3 debug/xbox_controller_6motors.py
```

This uses Pygame and the shared GPIO motor driver. To identify the controller's
button indices without driving the motors, append `--list-buttons`.
