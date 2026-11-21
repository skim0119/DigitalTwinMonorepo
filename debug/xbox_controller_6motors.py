import argparse
import sys
import time
from math import sqrt
from pathlib import Path

# Preserve the physical button assignments using the motor IDs in pins.json.
BUTTON_TO_MOTOR = {
    0: 4,  # A
    1: 2,  # B
    2: 3,  # X
    3: 5,  # Y
    6: 1,  # Back/View
    7: 0,  # Start/Menu
}
DIRECTION_MODIFIER_BUTTON = 4  # LB: held = backward, released = forward
CONTINUOUS_BUTTON = 5  # RB: confirm with print_button_indices()
RT_THRESHOLD = 16000  # SDL trigger range is 0 to 32767.
RAMP_TIME_S = 0.4  # Time from stopped to full jog speed (30 RPM).
MAX_STEP_RATE = 800  # 30 RPM at 1600 steps per revolution.
TAP_STEPS = round(0.13 * MAX_STEP_RATE)


def move_fixed(motors, motor, direction, instance_id, pause=time.sleep):
    target = motors.status()[motor][1] + direction * TAP_STEPS
    try:
        while motors.manual_command(motor, direction * MAX_STEP_RATE, target) != target:
            if any(event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id
                   for event in pygame.event.get(pygame.JOYDEVICEREMOVED)):
                return False
            pause(0.005)
        return True
    finally:
        motors.manual_command(motor, 0)


def return_trigger_held():
    return gamepad is not None and gamepad.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) >= RT_THRESHOLD


def jog_motor(motors, motor_button, motor_index, instance_id, *, return_on_release=False, pause=time.sleep):
    start = motors.status()[motor_index][1]
    speed = 0.0
    last_update = time.monotonic()
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
                    outbound = motors.manual_command(motor_index, 0) - start
                    speed = 0.0
                    last_update = now
                    print(f"Motor {motor_index}: outbound {outbound:+d} STEP pulses", flush=True)

            if outbound is None:
                target_speed = -1.0 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1.0
                step_target = None
            else:
                remaining = start - motors.status()[motor_index][1]
                if remaining == 0:
                    return True
                limit = min(1.0, sqrt(2 * abs(remaining) / (MAX_STEP_RATE * RAMP_TIME_S)))
                target_speed = limit if remaining > 0 else -limit
                step_target = start
            if speed * target_speed < 0:
                target_speed = 0.0
            change = (now - last_update) / RAMP_TIME_S
            speed += max(-change, min(change, target_speed - speed))
            last_update = now
            motors.manual_command(motor_index, speed * MAX_STEP_RATE, step_target)
            pause(0.005)
    finally:
        position = motors.manual_command(motor_index, 0) - start
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


def main(recording=None):
    global pygame, joy, gamepad
    parser = argparse.ArgumentParser(description="Jog the six Spirob motors with an Xbox controller.")
    parser.add_argument("--list-buttons", action="store_true", help="Print button indices without driving motors.")
    if recording is not None:
        recording.add_arguments(parser)
    args = parser.parse_args()

    if recording is not None and not args.list_buttons:
        recording.configure(parser, args)

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
    from control_core import open_manual_controller

    motors = open_manual_controller(Path(__file__).resolve().parents[1] / "pins.json",
                                    max_rate=MAX_STEP_RATE)
    if gamepad is None:
        print("SDL controller mapping unavailable; RT return mode is disabled.")
    returned_button = None
    try:
        pause = time.sleep if recording is None else recording.start(motors)
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
                                         return_on_release=return_mode, pause=pause):
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
                            if not move_fixed(motors, motor_index, direction, instance_id, pause):
                                sys.exit()
            pause(0.005)
    except KeyboardInterrupt:
        print("Main loop interrupted")
    finally:
        motors.stop()
        if recording is not None:
            recording.finish()


if __name__ == "__main__":
    main()
