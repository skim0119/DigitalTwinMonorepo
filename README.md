# Spirob

Tools for controlling the Spirob arm and monitoring its Raspberry Pi.

| Tool | Purpose |
| --- | --- |
| [`Spirob_Code/`](Spirob_Code/README.md) | Python arm control, including motor drivers, force PID, Xbox jogging, and experiment scripts. |
| [`stream-gpio/`](stream-gpio/README.md) | C++ service on the Pi that streams GPIO levels and can read HX711 load cells for monitoring and force control. |
| [`stream-gpio-monitor/`](stream-gpio-monitor/README.md) | Rust monitor that connects to the Pi over SSH and serves a browser dashboard. |
| [`push`](push) and [`pull`](pull) | Sync tracked files with the Pi. Both support `--dry-run`. |

In the force-control setup, the Pi service reads the HX711s, Python PID control
fetches readings over local UDP, and the monitor displays telemetry from the
WebSocket stream.
