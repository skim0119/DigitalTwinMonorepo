import time
import sys
sys.path.append('/home/mgazzola/spirob_py_code/Spirob_Code/core_nodes/motor')
from motors import Motors
import pygame
from pygame import joystick

pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
joy = pygame.joystick.Joystick(0)
joy.init()

# ----------------------------------------------------------------------
# Pin setup for 6 motors.
# The first 4 entries match your existing wiring. FILL IN the GPIO pin
# numbers for the two new motors (positions 4 and 5) based on how you've
# actually wired them.
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
RAMP_TIME_S = 0.4  # Time from stopped to full jog speed (30 RPM).
DIR_GUARD_S = 10e-6  # Match the provisional guard used in PID_multi.py.


def make_motors():
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


def jog_motor(motors, motor_button, motor_index, instance_id):
    max_steps_per_sec = motors.max_rpm * motors.steps_per_rev / 60
    speed = 0.0
    direction = None
    last_update = time.monotonic()
    step_progress = 0.0
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id:
                    return False
            if not (joy.get_button(CONTINUOUS_BUTTON) and joy.get_button(motor_button)):
                return True

            now = time.monotonic()
            target = -1.0 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1.0
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
            time.sleep(motors.pulse_width)
            motors._pul_motor_off(motor_index)
            time.sleep(motors.pulse_width)
    finally:
        motors._pul_motor_off(motor_index)


def print_button_indices():
    """
    Run this once (e.g. `python -c "from xbox_controller_6motors import print_button_indices; print_button_indices()"`),
    press each controller button you plan to use, and note the index it
    prints. Use those numbers to fill in BUTTON_TO_MOTOR above.
    """
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
    instance_id = joy.get_instance_id()
    motors = make_motors()
    try:
        while True:
            events = pygame.event.get()
            if any(event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id
                   for event in events):
                break
            if joy.get_button(CONTINUOUS_BUTTON):
                for button, motor_index in BUTTON_TO_MOTOR.items():
                    if joy.get_button(button):
                        if not jog_motor(motors, button, motor_index, instance_id):
                            sys.exit()
                        break
            else:
                for event in events:
                    if event.type == pygame.JOYBUTTONDOWN and event.instance_id == instance_id:
                        motor_index = BUTTON_TO_MOTOR.get(event.button)
                        if motor_index is not None and not joy.get_button(CONTINUOUS_BUTTON):
                            direction = -1 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1
                            main_loop(motors, motor_index, direction)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("Main loop interrupted")
    finally:
        motors.close()
