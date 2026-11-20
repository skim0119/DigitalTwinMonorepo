"""
Plot the CSV produced by read_three_loadcells.py
(columns: elapsed_s, motor1, motor2, motor3)

Usage:
    python3 plot_three_loadcells.py
    python3 plot_three_loadcells.py --csv /path/to/loadcell_readings.csv
"""

import argparse
import csv
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description="Plot load cell CSV data")
parser.add_argument('--csv', default="loadcell_readings_Nov_all.csv",
                     help="Path to the CSV file to plot")
args = parser.parse_args()

elapsed = []
motor0 = []
motor1 = []
motor2 = []
motor3 = []
motor4 = []
motor5 = []

with open(args.csv, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        elapsed.append(float(row["elapsed_s"]))
        for col, bucket in (("motor0", motor0), ("motor1", motor1), ("motor2", motor2), ("motor3", motor3), ("motor4", motor4) , ("motor5", motor5)):
            val = row[col]
            bucket.append(float(val) if val not in ("", "None") else None)

plt.figure()
plt.plot(elapsed, motor0, label="motor0", marker='.')
plt.plot(elapsed, motor1, label="motor1", marker='.')
plt.plot(elapsed, motor2, label="motor2", marker='.')
plt.plot(elapsed, motor3, label="motor3", marker='.')
plt.plot(elapsed, motor4, label="motor4", marker='.')
plt.plot(elapsed, motor5, label="motor5", marker='.')
plt.xlabel("Time (s)")
plt.ylabel("Raw ADC value")
plt.title("Load Cell Readings Over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("Loadcells_all_test_final.png")
