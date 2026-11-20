from hx711 import HX711
import RPi.GPIO as GPIO
import time
import argparse
import csv
 
# Shared clock pin -- same SCK line goes to all 3 HX711 modules
SCK = 6
 
# One DOUT pin per motor -- CHANGE motor2/motor3 to your real wiring
DOUT_M0 = 5
DOUT_M1 = 21
DOUT_M2 = 13
DOUT_M3 = 19
 
MOTOR_NAMES = ["motor0","motor1", "motor2", "motor3"]
DOUT_PINS = [DOUT_M0, DOUT_M1, DOUT_M2, DOUT_M3]
 
OUTPUT_CSV = "loadcell_readings_all.csv"
 
parser = argparse.ArgumentParser(description="Read 3 load cells and log samples to CSV")
parser.add_argument('--duration', type=float, default=None,
                     help="Seconds to run. Omit to run until Ctrl+C.")
args = parser.parse_args()
 
GPIO.setmode(GPIO.BCM)
 
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
 
    start_time = time.time()
 
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s"] + MOTOR_NAMES)
 
        print(f"Logging to {OUTPUT_CSV} ... press Ctrl+C to stop.")
 
        while True:
            elapsed_time = time.time() - start_time
            if args.duration is not None and elapsed_time >= args.duration:
                break
 
            values = []
            for hx in cells:
                #measurement = sum(hx.get_raw_data(times=2)) / 2
                measurement = hx.get_raw_data_mean(readings=5)
                values.append(measurement if measurement is not False else None)
 
            writer.writerow([f"{elapsed_time:.6f}"] + values)
            f.flush()
 
            #print(elapsed_time, values)
 
finally:
    GPIO.cleanup()
    print(f"Saved readings to {OUTPUT_CSV}")
