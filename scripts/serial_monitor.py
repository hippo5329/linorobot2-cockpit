#!/usr/bin/env python3
# ==============================================================================
# serial_monitor.py — stream a board's serial output into the cockpit terminal
#
# The Hardware Tests panel used to run `python3 -m serial.tools.miniterm`, which
# cannot work here: miniterm builds a Console() in its constructor, that calls
# termios.tcgetattr() on stdin, and the supervisor gives every command a PIPE.
# So every Monitor click ended in
#
#     termios.error: (25, 'Inappropriate ioctl for device')
#
# and the diagnostic applications -- test_sensors, i2c_detect, test_motors and
# the rest -- had no way to show their output at all. They print to serial and
# nothing was reading it.
#
# This reads the port and writes lines to stdout, unbuffered, which is exactly
# what the SSE runner forwards to the browser. No terminal, no tty, no cursor
# control: a pipe is all it needs.
# ==============================================================================
import argparse
import os
import signal
import sys
import time

try:
    import serial
except ImportError:
    print("[monitor] pyserial is not installed on this machine", flush=True)
    sys.exit(2)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port")
    ap.add_argument("baud", nargs="?", type=int, default=921600)
    # A board that was just flashed is REBOOTING, so its port disappears and
    # comes back. Opening once and giving up turns "monitor right after flash"
    # -- the whole point of the panel -- into a guaranteed failure.
    ap.add_argument("--wait", type=float, default=20.0,
                    help="seconds to wait for the port to appear")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long (0 = until stopped)")
    a = ap.parse_args()

    stop = {"now": False}

    def _stop(signum, frame):
        stop["now"] = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    deadline = time.time() + a.wait
    ser = None
    while time.time() < deadline and not stop["now"]:
        try:
            ser = serial.Serial(a.port, a.baud, timeout=0.5)
            break
        except Exception:
            time.sleep(0.4)
    if ser is None:
        print(f"[monitor] {a.port} did not appear within {a.wait:.0f}s", flush=True)
        return 1

    print(f"[monitor] {a.port} @ {a.baud} — streaming until you press Stop", flush=True)
    end = time.time() + a.seconds if a.seconds else None
    buf = bytearray()
    try:
        while not stop["now"] and (end is None or time.time() < end):
            try:
                chunk = ser.read(4096)
            except Exception as exc:
                # The board rebooting mid-stream is normal, not fatal.
                print(f"[monitor] port dropped ({exc}); waiting for it to come back",
                      flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                back = time.time() + a.wait
                while time.time() < back and not stop["now"]:
                    try:
                        ser = serial.Serial(a.port, a.baud, timeout=0.5)
                        break
                    except Exception:
                        time.sleep(0.4)
                if ser is None:
                    print("[monitor] the port did not come back", flush=True)
                    return 1
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            # Whole lines only, so a half-written line does not arrive as its
            # own SSE frame and wrap oddly in the terminal.
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                sys.stdout.write(line.decode("utf-8", "replace").rstrip("\r") + "\n")
            sys.stdout.flush()
    finally:
        if buf:
            sys.stdout.write(buf.decode("utf-8", "replace") + "\n")
            sys.stdout.flush()
        try:
            if ser:
                ser.close()
        except Exception:
            pass
    print("[monitor] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
