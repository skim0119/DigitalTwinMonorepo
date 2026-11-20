"""Native force and motion control for the Spirob Pi."""

import json
from pathlib import Path

from ._native import Controller


def open_controller(config_path, step_bounds, *, port=5050, max_force, max_rate, simulate=False,
                    motor=None, max_tightening_steps=0):
    """Open native workers from a settings file or a loaded settings dictionary."""
    config = config_path if isinstance(config_path, dict) else json.loads(Path(config_path).read_text())
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
        -1 if motor is None else motor, max_tightening_steps,
    )
