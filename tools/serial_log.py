"""serial_log.py - record a robot run to docs/testdata/ for the report.

Usage (from the repo root, with PlatformIO's Python which has pyserial):
    & "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/serial_log.py COM3 avoid_run1 60
    ... COM3 <name> <seconds> [keys to send at start] [keys to send at end]

Writes two files, named <date>_<time>_<name>:
    .log  everything the robot printed (boot log, events, telemetry)
    .tsv  only the telemetry table (header + rows) - opens straight in Excel
Also echoes to the terminal. Ctrl+C stops early and still saves.
"""
import datetime
import os
import sys
import time

import serial

BAUD = 115200


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    port, name, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
    send_start = sys.argv[4] if len(sys.argv) > 4 else ""
    send_end = sys.argv[5] if len(sys.argv) > 5 else ""

    out_dir = os.path.join(os.path.dirname(__file__), "..", "docs", "testdata")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    base = os.path.join(out_dir, f"{stamp}_{name}")

    s = None
    for _ in range(20):                      # the port vanishes briefly after a flash
        try:
            s = serial.Serial(port, BAUD, timeout=0.2)
            break
        except serial.SerialException:
            time.sleep(0.5)
    if s is None:
        print(f"could not open {port} - is a serial monitor still open?")
        sys.exit(1)

    time.sleep(0.3)
    if send_start:
        s.write(send_start.encode())

    lines, buf = [], ""
    end = time.time() + secs
    try:
        while time.time() < end:
            buf += s.read(4096).decode(errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.rstrip("\r")
                lines.append(line)
                print(line, flush=True)
    except KeyboardInterrupt:
        print("-- stopped early")
    finally:
        if send_end:
            s.write(send_end.encode())
        s.close()

    with open(base + ".log", "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # telemetry = the tab-separated header starting "ms" + numeric rows under it
    header = next((l for l in lines if l.startswith("ms\t")), None)
    rows = [l for l in lines if "\t" in l and l.split("\t", 1)[0].isdigit()]
    if header and rows:
        with open(base + ".tsv", "w", encoding="utf-8") as f:
            f.write(header + "\n" + "\n".join(rows) + "\n")

    print(f"\nsaved {base}.log" + (f" and .tsv ({len(rows)} rows)" if header and rows else ""))


if __name__ == "__main__":
    main()
