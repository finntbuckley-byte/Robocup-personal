"""crane_endurance.py - run N full crane cycles on the servotest build, log
them, and prompt you to check the servo every 10 cycles.

NOTE: the firmware moves the servo open-loop (by time, no position feedback),
so logged cycle times stay ~constant even if the servo is struggling.
Judge heat by TOUCH and SOUND: warm is fine; too hot to hold a finger on,
buzzing/humming while resting on the dummy, or the arm visibly not reaching
its positions = stop, then try 120 deg / slower speed / rest between cycles.

Usage (servotest build flashed, Serial Monitor CLOSED):
    & "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/crane_endurance.py COM3 25

Put a plain plastic dummy (no steel insert) under the arm: the arm lands on it
every cycle (the stall we're testing) but the magnets can't carry it away.
Saves docs/testdata/<date>_<time>_crane_endurance.log. Ctrl+C stops safely.
"""
import datetime
import os
import re
import sys
import time

import serial


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    target = int(sys.argv[2]) if len(sys.argv) > 2 else 25

    try:
        s = serial.Serial(port, 115200, timeout=0.1)
    except serial.SerialException as e:
        print(f"could not open {port} - close the Serial Monitor first ({e})")
        sys.exit(1)

    out_dir = os.path.join(os.path.dirname(__file__), "..", "docs", "testdata")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    path = os.path.join(out_dir, stamp + "_crane_endurance.log")

    print(f"running {target} cycles (about {target * 5} s) - Ctrl+C to stop early")
    s.write(b"l\n")
    t0 = time.time()
    buf, lines, count = "", [], 0
    try:
        while count < target:
            buf += s.read(4096).decode(errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.rstrip("\r")
                lines.append(f"{time.time() - t0:7.1f}s  {line}")
                if re.search(r"\[cycle\] done in \d+ ms", line):
                    count += 1
                    print(f"cycle {count:3d} done   {time.time() - t0:5.0f} s elapsed", flush=True)
                    if count % 10 == 0:
                        print("   >>> CHECK NOW: feel the servo (warm ok, too hot to hold = stop),"
                              " listen for buzzing, watch it still reaches pickup and drop")
    except KeyboardInterrupt:
        print("-- stopped early")
    finally:
        s.write(b"\n")          # any line stops the loop after the current cycle
        time.sleep(6)           # let the last cycle finish and park at rest
        s.close()

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{count} full cycles completed. Saved {path}")


if __name__ == "__main__":
    main()
