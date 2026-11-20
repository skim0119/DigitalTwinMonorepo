from hx711 import HX711
import RPi.GPIO as GPIO
import time
import argparse
import csv

# Shared clock pin -- same SCK line goes to all 3 HX711 modules
SCK = 6

# One DOUT pin per motor -- CHANGE motor2/motor3 to your real wiring
DOUT_M1 = 5
DOUT_M2 = 13
DOUT_M3 = 19

MOTOR_NAMES = ["motor1", "motor2", "motor3"]
DOUT_PINS = [DOUT_M1, DOUT_M2, DOUT_M3]

OUTPUT_CSV = "loadcell_readings.csv"

parser = argparse.ArgumentParser(description="Read 3 load cells and log samples to CSV")
parser.add_argument('--duration', type=float, default=None,
                     help="Seconds to run. Omit to run until Ctrl+C.")
parser.add_argument('--readings', type=int, default=5,
                     help="Samples averaged per read, per load cell. Lower = faster, noisier. Default 10.")
parser.add_argument('--print-every', type=int, default=5,
                     help="Only print to console every N rows, to cut SSH/terminal overhead. Default 10.")
args = parser.parse_args()

GPIO.setmode(GPIO.BCM)

start_time = time.perf_counter()

row_count = 0
try:
    cells = []
    for dout in DOUT_PINS:
        hx = HX711(
            dout_pin=dout,
            pd_sck_pin=SCK,
            select_channel='A',
            gain_channel_A=64
        )
        hx.reset()
        cells.append(hx)

    print("Load cells ready.")
    print("Tip: if this is still too slow, check the RATE pin on each HX711 board -- "
          "tying it high switches the chip from 10 SPS to 80 SPS (8x faster reads).")

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s"] + MOTOR_NAMES)

        print(f"Logging to {OUTPUT_CSV} ... press Ctrl+C to stop.")

        while True:
            elapsed_time = time.perf_counter() - start_time
            if args.duration is not None and elapsed_time >= args.duration:
                break

            values = []
            for hx in cells:
                measurement = hx.get_raw_data_mean(readings=args.readings)
                values.append(measurement if measurement is not False else None)

            writer.writerow([f"{elapsed_time:.6f}"] + values)
            row_count += 1
            print(row_count)
            if row_count % args.print_every == 0:
                f.flush()
                print(elapsed_time, values)
            time.sleep(0.001)

finally:
    GPIO.cleanup()
    print(f"Saved readings to {OUTPUT_CSV} | {time.perf_counter() - start_time} sec | {row_count=}")
