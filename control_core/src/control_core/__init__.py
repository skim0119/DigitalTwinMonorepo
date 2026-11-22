"""Native force and motion control for the Spirob Pi."""

import json
import math
from pathlib import Path


def safety_limits(config):
    """Read global limits, allowing older alternate pin maps to omit them."""
    keys = ("max_force_kgf", "max_step_rate")
    if not all(key in config for key in keys):
        defaults = json.loads(Path("pins.json").read_text())
        config = {**defaults, **config}
    force, rate = (float(config[key]) for key in keys)
    if not all(math.isfinite(value) and value > 0 for value in (force, rate)):
        raise ValueError("Global force and step-rate limits must be finite and positive")
    return force, rate


def open_controller(config_path, step_bounds, *, port=5050, max_force=None, max_rate=None, simulate=False,
                    motor=None, max_tightening_steps=0, manual_motion=False, skip_motor=None):
    """Open native workers from a settings file or a loaded settings dictionary."""
    config = config_path if isinstance(config_path, dict) else json.loads(Path(config_path).read_text())
    from ._native import Controller

    force_limit, rate_limit = safety_limits(config)
    max_force = force_limit if max_force is None else max_force
    max_rate = rate_limit if max_rate is None else max_rate
    if not (0 < max_force <= force_limit and 0 < max_rate <= rate_limit):
        raise ValueError("Requested force and step-rate limits must be positive and within global limits")
    motors = config["motors"]
    if [motor["motor"] for motor in motors] != list(range(6)) or len(step_bounds) != 6:
        raise ValueError("Expected motors 0 through 5 and six step ranges")
    pins = [motor[key] for motor in motors for key in ("pul", "dir", "dout", "sck")]
    if any(type(pin) is not int or not 0 <= pin <= 27 for pin in pins) or len(set(pins)) != len(pins):
        raise ValueError("GPIO assignments must be distinct BCM pins 0 through 27")
    return Controller(
        [
            (motor["pul"], motor["dir"], *(motor["pid"][key] for key in ("kp", "ki", "kd")), *bounds)
            for motor, bounds in zip(motors, step_bounds)
        ],
        port, max_force, max_rate, config["pid_deadband_g"],
        config.get("pid_filter_alpha", 0.4), simulate,
        -1 if motor is None else motor, max_tightening_steps, manual_motion,
        -1 if skip_motor is None else skip_motor,
        config.get("pid_max_acceleration_steps_s2", 0),
        [motor["pid"].get("kff", 0) for motor in motors],
        config.get("pid_output_filter_ms", 0),
    )


def open_manual_controller(config_path, *, max_rate=800, simulate=False, motor=None):
    """Open a bench controller with session-relative pulse counts and no HX711 input."""
    # The bench tools have no calibrated arm reference. These are counter bounds.
    bounds = [(-2**31, 2**31 - 1)] * 6
    return open_controller(config_path, bounds, max_rate=max_rate,
                           simulate=simulate, motor=motor, manual_motion=True)
