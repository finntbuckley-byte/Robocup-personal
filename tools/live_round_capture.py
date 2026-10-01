"""Passive serial recording: flush every line; no commands sent to the robot."""
import datetime
import time
from pathlib import Path

import serial

path = Path(__file__).resolve().parent.parent / "logs"
path.mkdir(exist_ok=True)
log = path / ("arena-live-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".log")
started = False
finished_at = None
deadline = time.monotonic() + 900
with serial.Serial("COM3", 115200, timeout=0.3) as port, log.open("w", encoding="utf-8", buffering=1) as out:
    print(f"RECORDING COM3; passive only; log={log}", flush=True)
    pending = b""
    try:
        while time.monotonic() < deadline:
            chunk = port.read(port.in_waiting or 1)
            if not chunk:
                if finished_at is not None and time.monotonic() - finished_at > 8:
                    break
                continue
            pending += chunk
            while b"\n" in pending:
                raw, pending = pending.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                out.write(line + "\n")
                if "ROUND START" in line or "round=HOME" in line or "round=RUN" in line:
                    started = True
                if started and ("ROUND OVER" in line or "ROUND STOPPED" in line or "round=OVER" in line):
                    if finished_at is None:
                        finished_at = time.monotonic()
                if any(tag in line for tag in ("HOME", ">>>", "!!", "[collection]", "SUM", "home base colour")):
                    print(line, flush=True)
            if finished_at is not None and time.monotonic() - finished_at > 8:
                break
    except (KeyboardInterrupt, serial.SerialException) as exc:
        print(f"RECORDING ENDED: {exc}", flush=True)
    finally:
        if pending:
            out.write(pending.decode("utf-8", errors="replace"))
print(f"SAVED {log}", flush=True)
