# Spirob control core

C++ force and displacement control for the Raspberry Pi 4, exposed to Python
through pybind11. The experiment runner sends targets and logs telemetry. Six
native workers calculate motor commands and generate STEP/DIR edges.

From the repository root, install into the experiment environment:

```sh
make install
```

Source installation needs a C++17 compiler. Use `uv build control_core --wheel`
to build a wheel for the Pi's architecture and Python version.

Rebuild and start the Pi streamer before opening a live trial.
It owns the HX711 GPIO and serves unaveraged calibrated conversions over local
UDP. Each native worker processes fresh timestamps and applies the EMA
configured in root `pins.json`. The existing monitor still receives averaged
values. Stop any standalone motor or HX711 scripts before a live trial.

Launch the direct collector from the repository root so it can read `pins.json`,
or set `SPIROB_PINS_CONFIG` to the full path. The global limits in
[pins.json](../pins.json) default to 4.5 kgf and 3000 steps/s. The direct collector accepts a lower per-trial tension limit. Native workers stop on an expired
command, stale sensor data, or measured tension at the configured limit.
An invalid conversion pauses STEP pulses until two valid conversions arrive.
A silent sensor pauses motion after 250 ms without a usable conversion.
The controller ends the session after one second without usable feedback.
In force mode, `pid_max_acceleration_steps_s2` limits changes in commanded
step rate and brings the rate to zero before reversing. The shared pin map
sets it to 1500 pulses/s². Omit it or set it to zero to disable slew limiting.
Force limits, expired commands and sensor pauses still stop steps immediately.

The force PID filters the bounded step-rate request before slew limiting.
`pid_output_filter_ms` is the first-order time constant (50 ms in the pin map,
zero to disable). Sensor pauses clear the filter. PID Multi CSVs include the
weighted P, I, D and feedforward contributions, bounded request, filtered rate
and applied step rate, all in pulses/s at the telemetry sampling interval.

Each motor's `pid.kff` defaults to zero. Trajectory callers can pass six
planned target slopes in kg/s through the optional `target_rates` command
argument. The controller smooths those rates with `pid_filter_alpha` and
adds their feedforward contribution before applying the speed and slew limits.
Existing step-and-hold callers use zero target rate.
Pulse timing and stop latency need to be measured on the Pi before enabling
motor drivers for a live run.

Run the simulated GPIO checks with
`python -m unittest discover -s control_core/tests` after installation.


## Manual motion

The Xbox and single-motor CLI tools use `open_manual_controller`. It acquires
the same GPIO lock as the experiment controller. Manual motion runs without the
HX711 streamer and uses session-relative pulse counts. It has no measured
tension protection or calibrated travel limits.

`manual_command(motor, rate, target=None)` takes a signed rate in steps per
second and an optional absolute pulse count within the session. It returns the
motor's current count. Only one motor can move at a time. Refresh an active
command within 200 ms or the controller stops and must be reopened. A zero rate
stops motion and returns the count at the stop.
