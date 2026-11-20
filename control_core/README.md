# Spirob control core

C++ force and displacement control for the Raspberry Pi 4, exposed to Python
through pybind11. The experiment API sends targets and logs telemetry. Six
native workers calculate motor commands and generate STEP/DIR edges.

From the repository root, install into the experiment API environment:

```sh
python -m pip install ./control_core ./experiment-api
```

Source installation needs a C++17 compiler. Use `uv build control_core --wheel`
to build a wheel for the Pi's architecture and Python version.

Rebuild and start the Pi streamer before opening a live trial.
It owns the HX711 GPIO and serves unaveraged calibrated conversions over local
UDP. Each native worker processes fresh timestamps and applies the EMA
configured in root `pins.json`. The existing monitor still receives averaged
values. Stop any standalone motor or HX711 scripts before a live trial.

Launch the experiment API from the repository root so it can read `pins.json`,
or set `SPIROB_PINS_CONFIG` to the full path. The API calibration file supplies
verified force, travel, and speed limits. Native workers stop on an expired
command, stale sensor data, or measured tension at the configured limit.
Pulse timing and stop latency need to be measured on the Pi before enabling
motor drivers for a live run.

Run the simulated GPIO checks with
`python -m unittest discover -s control_core/tests` after installation.
