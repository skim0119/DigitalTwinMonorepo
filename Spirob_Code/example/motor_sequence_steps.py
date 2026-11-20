import time
import sys
sys.path.append('/home/mgazzola/spirob_py_code/Spirob_Code/core_nodes/motor')
from motors import Motors

PUL = [10, 18, 4, 23, 22, 26]
DIR = [9, 14, 27, 24, 25, 16]
MOTOR_ORIENTATION = [True, True, True, True, True, True]

ROUNDS_PER_MOTOR = 1
STEPS_PER_ROUND = 800        # number of steps each round runs
DELAY_BETWEEN_ROUNDS = 10    # seconds between rounds on the same motor
DELAY_BETWEEN_MOTORS = 10    # seconds between finishing one motor and starting the next
DIRECTION = -1                # 1 = forward, -1 = backward
SPEED = 1


def run_motor_steps(motors, motor_index, direction=-1, speed=1, steps=100):
    print('Running motor', motor_index, 'forward' if direction > 0 else 'backward', 'at speed', speed, 'for', steps, 'steps')
    if direction > 0:
        motors.dir_motor_forward([motor_index])
    else:
        motors.dir_motor_backward([motor_index])
    motors.set_motor_speed([motor_index], speed)
    motors.start_motor([motor_index])

    start_steps = motors.num_of_steps[motor_index]
    try:
        while abs(motors.num_of_steps[motor_index] - start_steps) < steps:
            motors.update([motor_index])
    finally:
        motors.stop_motor([motor_index])


def main():
    motors = Motors(
        PUL,
        DIR,
        MOTOR_ORIENTATION,
        steps_per_rev=1600,
        max_rpm=30,
        pulse_width=10 * 1e-6,
        debug=False)

    try:
        for motor_index in range(4):
            for round_num in range(ROUNDS_PER_MOTOR):
                print(f"Motor {motor_index}, round {round_num + 1}/{ROUNDS_PER_MOTOR}")
                run_motor_steps(motors, motor_index, DIRECTION, SPEED, STEPS_PER_ROUND)

                if round_num < ROUNDS_PER_MOTOR - 1:
                    print(f"Waiting {DELAY_BETWEEN_ROUNDS}s before next round...")
                    time.sleep(DELAY_BETWEEN_ROUNDS)

            if motor_index < 3:
                print(f"Waiting {DELAY_BETWEEN_MOTORS}s before next motor...")
                time.sleep(DELAY_BETWEEN_MOTORS)

    except KeyboardInterrupt:
        print("\nInterrupted by Ctrl+C.")
    finally:
        motors.stop_motor([0, 1, 2, 3])


if __name__ == "__main__":
    main()
