"""notch_view.py - live readout of the notch ToF (index 2) for angling it.

Run with the nav build flashed and the Serial Monitor CLOSED:
    & "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/notch_view.py COM3

Prints ~5 lines a second: distance, a bar, the sensor status, and which band
the reading falls in. Ctrl+C to stop (the last 20 s of readings are summarised).

Targets (see BENCH_TODO): empty = steady and highest (>= ~130 mm or a steady 0),
upright weight seated <= ~70, at the notch mouth >= seated + 15, lying <= ~110.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
try:
    s = serial.Serial(port, 115200, timeout=0.02)
except serial.SerialException as e:
    print(f"could not open {port} - close the Serial Monitor first ({e})")
    sys.exit(1)

s.write(b"x")             # motors stay killed while you work on the robot
time.sleep(0.1)
s.write(b"t")             # telemetry off so only the ToF table comes back
time.sleep(0.3)
s.read(65536)

print("notch ToF live - Ctrl+C to stop")
hist = []
try:
    while True:
        s.write(b"u")
        t = time.time()
        buf = ""
        while time.time() - t < 0.2:
            buf += s.read(4096).decode(errors="replace")
        for line in buf.splitlines():
            f = line.split("\t")
            if len(f) == 8 and f[0] == "2":
                raw = int(f[3]) if f[3].isdigit() else 0
                status, age, err = f[4], f[5], f[6]
                if age == "never" or err != "0":
                    label = "NO DATA (sensor down?)"
                elif status != "0":
                    label = f"rejected by code (status {status})"
                elif raw == 0:
                    label = "no target"
                elif raw <= 70:
                    label = "SEATED range"
                elif raw <= 110:
                    label = "weight in notch (not seated / lying)"
                elif raw < 130:
                    label = "grey zone"
                else:
                    label = "empty / far"
                bar = "#" * min(60, raw // 5)
                print(f"{raw:5d} mm  st={status:<3} {bar:<60} {label}", flush=True)
                hist.append((time.time(), raw, status))
except KeyboardInterrupt:
    pass
finally:
    s.write(b"t")          # telemetry back on
    s.close()

recent = [r for tm, r, st in hist if time.time() - tm < 20 and st == "0"]
if recent:
    print(f"\nlast 20 s (valid readings): min {min(recent)}  max {max(recent)}  "
          f"n={len(recent)}")
