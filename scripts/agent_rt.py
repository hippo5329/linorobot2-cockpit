#!/usr/bin/env python3
"""Run the micro-ROS agent with real-time settings, then exec it:

  agent_rt.py ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyHS1 -b 2000000

On a small robot computer the agent shares four cores with SLAM, Nav2 and a dozen Python
nodes. Starved for tens of milliseconds it cannot drain the UART -- at 2 Mbaud a 4 KB buffer
fills in ~20 ms -- frames are lost, the board's pings go unanswered and it drops the session
(an Arduino UNO Q under SLAM + Nav2, 2026-10-15). From the environment, each step best-effort:

  LINO_AGENT_RT_PRIO   SCHED_FIFO priority (1-99); needs CAP_SYS_NICE and an rtprio limit
  LINO_AGENT_CPU       pin to these CPUs ("3", "2,3")
  LINO_AGENT_LOW_LATENCY=1 (default)  ASYNC_LOW_LATENCY on the --dev serial port

Policy and affinity are inherited across exec and by every thread the agent starts. A step
that is not allowed is reported and skipped: the agent always starts.
"""
import fcntl
import os
import struct
import sys

TIOCGSERIAL, TIOCSSERIAL = 0x541E, 0x541F
ASYNC_LOW_LATENCY = 1 << 13
SERIAL_STRUCT_LEN = 72          # struct serial_struct on 64-bit Linux
FLAGS_OFFSET = 16               # type, line, port, irq: four ints, then flags


def say(msg):
    print(f"[agent_rt] {msg}", flush=True)


def parse_cpus(spec: str) -> set:
    out = set()
    for part in str(spec or "").replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(part))
    return out


def serial_dev(argv) -> str:
    for i, a in enumerate(argv[:-1]):
        if a == "--dev":
            return argv[i + 1]
    return ""


def low_latency(dev: str) -> str:
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        return f"low latency: {dev} not opened ({e.strerror})"
    try:
        buf = bytearray(SERIAL_STRUCT_LEN)
        fcntl.ioctl(fd, TIOCGSERIAL, buf, True)
        flags = struct.unpack_from("i", buf, FLAGS_OFFSET)[0]
        if flags & ASYNC_LOW_LATENCY:
            return f"low latency: already on for {dev}"
        struct.pack_into("i", buf, FLAGS_OFFSET, flags | ASYNC_LOW_LATENCY)
        fcntl.ioctl(fd, TIOCSSERIAL, buf)
        return f"low latency: on for {dev}"
    except OSError as e:
        return f"low latency: not set on {dev} ({e.strerror})"
    finally:
        os.close(fd)


def apply(env, argv) -> list:
    """The settings, in order; each a line to report. Raises nothing."""
    notes = []
    cpus = parse_cpus(env.get("LINO_AGENT_CPU", ""))
    if cpus:
        try:
            have = os.sched_getaffinity(0)
            use = cpus & have
            if use:
                os.sched_setaffinity(0, use)
                notes.append(f"pinned to CPU {','.join(map(str, sorted(use)))}")
            else:
                notes.append(f"CPU {sorted(cpus)} not available here ({sorted(have)}): not pinned")
        except OSError as e:
            notes.append(f"not pinned ({e.strerror})")
    prio = str(env.get("LINO_AGENT_RT_PRIO", "") or "").strip()
    if prio:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(prio)))
            notes.append(f"SCHED_FIFO priority {int(prio)}")
        except (OSError, ValueError) as e:
            notes.append(f"SCHED_FIFO {prio} not set ({getattr(e, 'strerror', None) or e}): "
                         "the container needs CAP_SYS_NICE and an rtprio limit")
    dev = serial_dev(argv)
    if dev and str(env.get("LINO_AGENT_LOW_LATENCY", "1")) == "1":
        notes.append(low_latency(dev))
    return notes


def main(argv) -> None:
    if not argv:
        say("nothing to run")
        sys.exit(2)
    for note in apply(os.environ, argv):
        say(note)
    os.execvp(argv[0], argv)


if __name__ == "__main__":
    main(sys.argv[1:])
