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


def main_loop(motor_index: int = 0, dir: int = 1, speed: float = 1):
    motors = Motors(
        PUL,
        DIR,
        MOTOR_ORIENTATION,
        steps_per_rev=1600,
        max_rpm=30,
        pulse_width=10 * 1e-6,
        debug=False)

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
            motors.update([motor_index])
            if elapsed > duration:
                Action = False
    except KeyboardInterrupt:
        print("\nMain loop interrupted by Ctrl+C.")
    finally:
        # stops however many motors are configured, instead of a hardcoded [0,1,2,3]
        motors.stop_motor(list(range(len(PUL))))


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
    try:
        while True:
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    pressed_motor = BUTTON_TO_MOTOR.get(event.button)
                    if pressed_motor is None:
                        continue
                    direction = -1 if joy.get_button(DIRECTION_MODIFIER_BUTTON) else 1
                    main_loop(pressed_motor, direction, 1)
                if event.type == pygame.JOYBUTTONUP:
                    pass
    except KeyboardInterrupt:
        print("Main loop interrupted")
    finally:
        pass
