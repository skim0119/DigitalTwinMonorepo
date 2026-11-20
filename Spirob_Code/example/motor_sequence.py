import time
import sys
sys.path.append('/home/mgazzola/spirob_py_code/Spirob_Code/core_nodes/motor')
from motors import Motors

PUL = [17, 23, 22, 26]
DIR = [27, 24, 25, 16]
MOTOR_ORIENTATION = [True, True, True, True]

ROUNDS_PER_MOTOR = 2
ROUND_DURATION = 0.3        # seconds each round runs
DELAY_BETWEEN_ROUNDS = 10   # seconds between rounds on the same motor
DELAY_BETWEEN_MOTORS = 10   # seconds between finishing one motor and starting the next
DIRECTION = 1               # 1 = forward, -1 = backward
SPEED = 1


def run_motor(motors, motor_index, direction=-1, speed=1, duration=0.3):
    print('Running motor', motor_index, 'forward' if direction > 0 else 'backward', 'at speed', speed)
    if direction > 0:
        motors.dir_motor_forward([motor_index])
    else:
        motors.dir_motor_backward([motor_index])
    motors.set_motor_speed([motor_index], speed)
    motors.start_motor([motor_index])

    start_time = time.time()
    try:
        while time.time() - start_time < duration:
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
                run_motor(motors, motor_index, DIRECTION, SPEED, ROUND_DURATION)

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
