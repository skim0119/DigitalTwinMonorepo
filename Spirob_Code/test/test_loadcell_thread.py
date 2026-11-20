from hx711 import HX711
import RPi.GPIO as GPIO
from concurrent.futures import ThreadPoolExecutor
import time
import argparse
import csv

# Each load cell now has its OWN clock pin -- no more shared SCK line.
# Rewire motor1/motor2/motor3's PD_SCK to the new pins below (motor0 keeps its
# original clock pin, so only 3 wires need to move).
DOUT_PINS = [2, 5, 7, 21, 17, 12]
SCK_PINS  = [3, 6, 11, 13, 20, 19]   # <-- pick different free GPIOs here if these clash with anything

MOTOR_NAMES = ["motor0", "motor1", "motor2", "motor3", "motor4", "motor5"]

OUTPUT_CSV = "loadcell_readings_Nov_all.csv"

parser = argparse.ArgumentParser(description="Read 4 load cells (independent clock pins, read in parallel) and log samples to CSV")
parser.add_argument('--duration', type=float, default=None,
                     help="Seconds to run. Omit to run until Ctrl+C.")
args = parser.parse_args()

GPIO.setmode(GPIO.BCM)

try:
    cells = []
    for dout, sck in zip(DOUT_PINS, SCK_PINS):
        hx = HX711(
            dout_pin=dout,
            pd_sck_pin=sck,
            select_channel='A',
            gain_channel_A=64
        )
        hx.reset()
        cells.append(hx)

    print("Load cells ready.")

    def read_one(hx):
        measurement = hx.get_raw_data_mean(readings=5)
        return measurement if measurement is not False else None

    start_time = time.time()

    with open(OUTPUT_CSV, "w", newline="") as f, ThreadPoolExecutor(max_workers=len(cells)) as executor:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s"] + MOTOR_NAMES)

        print(f"Logging to {OUTPUT_CSV} ... press Ctrl+C to stop.")

        while True:
            elapsed_time = time.time() - start_time
            if args.duration is not None and elapsed_time >= args.duration:
                break

            # Each cell now has its own dedicated clock pin, so these can safely
            # run at the same time instead of one after another.
            values = list(executor.map(read_one, cells))

            writer.writerow([f"{elapsed_time:.6f}"] + values)
            f.flush()

finally:
    GPIO.cleanup()
    print(f"Saved readings to {OUTPUT_CSV}")
