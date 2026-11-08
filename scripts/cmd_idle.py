"""When a teleop publisher goes quiet (gamepad_publisher.py).

A teleop node republishes its command because a base stops when /cmd_vel goes quiet. Held at zero, though, it
keeps saying "stop" at its full rate for as long as it lives -- and a second publisher on /cmd_vel then fights
everything else that drives the robot: Nav2's commands reached the firmware interleaved with twice as many
zeros, and a walking robot covered a sixth of what it was told for hours (a Teleop tab's publisher left
running, 2026-11-08). So zero is said for `idle_s` (long enough for the base to hear it), then nothing until
the command moves again. Plain Python: no ROS, so the rule is testable on its own.
"""


class IdleGate:
    def __init__(self, idle_s):
        self.idle_s = idle_s    # <= 0: never go quiet (the old behaviour)
        self.zero_since = None  # when the command last became zero
        self.quiet = False

    def publish(self, cmd, now):
        """True while `cmd` (lx, ly, az, after the deadman) should be published at `now` (s, monotonic)."""
        if any(cmd):
            self.zero_since = None
            self.quiet = False
            return True
        if self.zero_since is None:
            self.zero_since = now
        if self.idle_s > 0 and now - self.zero_since >= self.idle_s:
            self.quiet = True
            return False
        return True
