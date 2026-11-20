import argparse
import sys
import time
from math import sqrt
from pathlib import Path

# ----------------------------------------------------------------------
# BCM pin pairs in the controller's button-mapping order.
# ----------------------------------------------------------------------
PUL = [4, 23, 22, 26, 10, 18]       
DIR = [27, 1, 25, 0, 9, 14]       
MOTOR_ORIENTATION = [True,True,True,True,True,True]

# ----------------------------------------------------------------------
# Button -> motor mapping.
# Your original code mapped A/B/X/Y to motors 2/0/1/3 (kept as-is below).
# Two more entries have been added for motors 4 and 5, using button
# indices 6 and 7 (commonly Back/View and Start/Menu) as placeholders.
#
# IMPORTANT: Xbox controller button indices are NOT consistent across
# platforms/drivers (wired vs Bluetooth, Windows vs Linux xpad/xboxdrv).
# Run print_button_indices() below once, press the buttons you want to
# use for motors 4/5, and update the indices here to match what it prints.
# ----------------------------------------------------------------------
BUTTON_TO_MOTOR = {
    0: 2,  # A
    1: 0,  # B
    2: 1,  # X
    3: 3,  # Y
    6: 4,  # Back/View -- CONFIRM with print_button_indices()
    7: 5,  # Start/Menu -- CONFIRM with print_button_indices()
}
DIRECTION_MODIFIER_BUTTON = 4  # LB: held = backward, released = forward
CONTINUOUS_BUTTON = 5  # RB: confirm with print_button_indices()
RT_THRESHOLD = 16000  # SDL trigger range is 0 to 32767.
RAMP_TIME_S = 0.4  # Time from stopped to full jog speed (30 RPM).
DIR_GUARD_S = 10e-6  # Match the provisional guard used by control_core.


def make_motors():
    motor_dir = Path(__file__).resolve().parents[1] / "Spirob_Code/core_nodes/motor"
    sys.path.insert(0, str(motor_dir))
    from motors import Motors

    return Motors(
        PUL,
        DIR,
        MOTOR_ORIENTATION,
        steps_per_rev=1600,
        max_rpm=30,
        pulse_width=10 * 1e-6,
        debug=False)

def main_loop(motors, motor_index: int = 0, dir: int = 1, speed: float = 1):
    print('Running motor', motor_index, 'forward' if dir > 0 else 'backward', 'at speed', speed)
    if dir > 0:
        motors.dir_motor_forward([motor_index])
    else:
        motors.dir_motor_backward([motor_index])
    motors.set_motor_speed([motor_index], speed)
    motors.start_motor([motor_index])

    print("Starting main loop.")
    Action = True
    start_time = time.time()
    duration = 0.13
    try:
        while Action:
            elapsed = time.time() - start_time
            edge = motors.update(motor_index)
            if elapsed > duration and edge == -1:
                Action = False
    except KeyboardInterrupt:
        print("\nMain loop interrupted by Ctrl+C.")
    finally:
        motors.stop_motor(motor_index)


def return_trigger_held():
    return gamepad is not None and gamepad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) >= RT_THRESHOLD


def jog_motor(motors, motor_button, motor_index, instance_id, *, return_on_release=False):
    max_steps_per_sec = motors.max_rpm * motors.steps_per_rev / 60
    speed = 0.0
    direction = None
    last_update = time.monotonic()
    step_progress = 0.0
    position = 0
    outbound = None
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id:
                    return False
            now = time.monotonic()
            if outbound is None:
                modifier_held = (return_trigger_held() if return_on_release
                                 else joy.get_button(CONTINUOUS_BUTTON))
                if not (modifier_held and joy.get_button(motor_button)):
                    if not return_on_release:
                        return True
                    outbound = position
                    speed = 0.0
                    step_progress = 0.0
                    last_update = now
                    print(f"Motor {motor_index}: outbound {outbound:+d} STEP pulses", flush=True)

            if outbound is None:
                target = -1.0 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1.0
            else:
                if position == 0:
                    return True
                # Slow down as the remaining return distance shrinks.
                limit = min(1.0, sqrt(2 * abs(position) / (max_steps_per_sec * RAMP_TIME_S)))
                target = -limit if position > 0 else limit
            if speed * target < 0:
                target = 0.0
            elapsed = now - last_update
            change = elapsed / RAMP_TIME_S
            speed += max(-change, min(change, target - speed))
            last_update = now

            if speed == 0.0:
                step_progress = 0.0
            step_progress += elapsed * abs(speed) * max_steps_per_sec
            if step_progress < 1.0:
                time.sleep(0.0005)
                continue

            forward = speed > 0
            if forward != direction:
                time.sleep(DIR_GUARD_S)
                if forward:
                    motors.dir_motor_forward(motor_index)
                else:
                    motors.dir_motor_backward(motor_index)
                time.sleep(DIR_GUARD_S)
                direction = forward

            # Drop missed steps after a scheduling delay instead of sending a burst.
            step_progress %= 1.0
            motors._pul_motor_on(motor_index)
            position += 1 if forward else -1
            time.sleep(motors.pulse_width)
            motors._pul_motor_off(motor_index)
            time.sleep(motors.pulse_width)
    finally:
        motors._pul_motor_off(motor_index)
        if outbound is not None:
            print(f"Motor {motor_index}: return {position - outbound:+d} STEP pulses, "
                  f"residual {position:+d}", flush=True)


def print_button_indices():
    """Print button indices for BUTTON_TO_MOTOR."""
    print("Press buttons to see their index (Ctrl+C to stop)...")
    try:
        while True:
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"button index: {event.button}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jog the six Spirob motors with an Xbox controller.")
    parser.add_argument("--list-buttons", action="store_true", help="Print button indices without driving motors.")
    args = parser.parse_args()

    import pygame
    from pygame._sdl2 import controller

    pygame.init()
    pygame.joystick.init()
    if not pygame.joystick.get_count():
        parser.error("Connect an Xbox controller first.")
    joy = pygame.joystick.Joystick(0)
    joy.init()
    if args.list_buttons:
        print_button_indices()
        sys.exit()

    controller.init()
    gamepad = controller.Controller(0) if controller.is_controller(0) else None
    instance_id = joy.get_instance_id()
    motors = make_motors()
    if gamepad is None:
        print("SDL controller mapping unavailable; RT return mode is disabled.")
    returned_button = None
    try:
        while True:
            events = pygame.event.get()
            if any(event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id
                   for event in events):
                break
            if returned_button is not None and not joy.get_button(returned_button):
                returned_button = None
            return_mode = return_trigger_held()
            if return_mode or joy.get_button(CONTINUOUS_BUTTON):
                for button, motor_index in BUTTON_TO_MOTOR.items():
                    if button != returned_button and joy.get_button(button):
                        if not jog_motor(motors, button, motor_index, instance_id,
                                         return_on_release=return_mode):
                            sys.exit()
                        if return_mode:
                            returned_button = button
                        break
            else:
                for event in events:
                    if event.type == pygame.JOYBUTTONDOWN and event.instance_id == instance_id:
                        motor_index = BUTTON_TO_MOTOR.get(event.button)
                        if (motor_index is not None and event.button != returned_button
                                and not joy.get_button(CONTINUOUS_BUTTON) and not return_trigger_held()):
                            direction = -1 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1
                            main_loop(motors, motor_index, direction)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("Main loop interrupted")
    finally:
        motors.close()
