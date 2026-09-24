"""A "RAN AWAY" verdict must survive arithmetic before it is printed.

On 2026-09-25, round ~180 of a 500-round soak, a healthy robot was reported as
having left the 6 m room:

    ❌ NAV2 LEG 1/2 RAN AWAY: asked for (3.00, 0.00), left the 6 m room instead;
       ended at (-5.73, +1.91) in map, odom pose (-5.73, +1.91),
       map->odom (+5.91, -1.69) m after 0 s

Three things in that line contradict each other, and the verdict rested on the one
that was wrong:

  * `after 0 s` -- nothing crosses 6 m of floor in zero seconds;
  * the map pose equals the odom pose EXACTLY while map->odom is nearly 6 m, so
    the identity `map_pose = odom_pose + (map->odom)` fails;
  * applying the offset gives (+0.18, +0.22) -- home, where the robot actually was.

The leg was judged in the first instant of the stack's life, before SLAM had
published a converged `map->odom`, so `map->base_link` resolved through an identity
offset. `tf_ok` was True, which is why the existing "TF could not answer" guard did
not catch it: TF answered, with a transform composed from a tree that had not
settled.

This is the same mistake as the soak that stopped a healthy run for measuring drift
in `map` instead of `/odom`, run in the opposite direction -- and it gets MORE
likely with time, not less. `/odom` wanders steadily from its origin over hundreds
of rounds while SLAM absorbs the difference, so the further a soak gets, the more
certainly any incoherent instant reads as a runaway. A 500-round soak would have
produced these indefinitely.

So the rule is not "check TF harder". It is: a verdict may not rest on a number
whose own frame cannot be corroborated by the other two numbers already in hand.
"""
import math
import os
import sys

import pytest

# The script exits at import without rclpy, and tests/test_nav2_goal_motion.py
# already owns the stubbing that gets around that. Reusing its loader keeps ONE
# implementation of "import this script without a ROS graph" -- a second copy
# would drift, and the drift would show up as this file mysteriously testing an
# older module than its neighbour.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_nav2_goal_motion import _load_module

tng = _load_module()


class _Pos:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Node:
    """The three readings _runaway judges, and nothing else."""

    def __init__(self, map_xy, odom_xy, offset, moved, tf_ok=True):
        self._last_xy = map_xy
        self.tf_ok = tf_ok
        self.odom_max_dist = moved
        self._offset = offset

        class _O:
            pass

        self.latest_odom = _O()
        self.latest_odom.pose = _O()
        self.latest_odom.pose.pose = _O()
        self.latest_odom.pose.pose.position = _Pos(*odom_xy)

    def map_odom_offset(self):
        return self._offset


# The measured numbers, verbatim from the false red.
FALSE_RED = dict(map_xy=(-5.73, 1.91), odom_xy=(-5.73, 1.91),
                 offset=(5.91, -1.69), moved=0.0)


def test_the_measured_false_red_is_no_longer_a_runaway():
    node = _Node(**FALSE_RED)
    assert math.hypot(*node._last_xy) > tng.RUNAWAY_RADIUS_M, \
        "the fixture must still be outside the room, or it proves nothing"
    assert tng._pose_is_coherent(node) is False, \
        "odom + offset lands at home, so the triple is incoherent and must say so"
    assert not tng._runaway(node), (
        "the 2026-09-25 false red would be printed again: a robot at home, "
        "reported as having left the room after 0 s")


def test_the_incoherence_is_named_in_the_line():
    """Rejecting the verdict is half the job; a reader must learn why the leg died."""
    node = _Node(**FALSE_RED)
    node.goal_frame = "map"
    text = tng._where(node)
    assert "INCOHERENT" in text, "the line does not say the readings disagree"
    # The corroborating position, so the reader sees home without doing the sum.
    assert "+0.18" in text and "+0.22" in text, \
        "the line must show where odom+offset actually puts the robot"


def test_a_real_runaway_is_still_caught():
    """The guard must not have bought quiet at the cost of the thing it protects.

    A genuine runaway: the base's own wheels reported crossing the floor, and the
    three readings agree about where it ended up.
    """
    node = _Node(map_xy=(-7.0, 1.0), odom_xy=(-7.2, 1.1), offset=(0.2, -0.1), moved=7.1)
    assert tng._pose_is_coherent(node) is True
    assert tng._runaway(node), "a coherent, physically possible runaway must fire"


def test_a_pose_jump_without_wheel_travel_is_not_a_runaway():
    """Coherent readings, but the base never moved: the estimate jumped.

    The base cannot be fooled about its own wheels, which is why odom_max_dist is
    recorded separately from any TF-derived number.
    """
    node = _Node(map_xy=(-7.0, 1.0), odom_xy=(-7.2, 1.1), offset=(0.2, -0.1), moved=0.3)
    assert tng._pose_is_coherent(node) is True, "this fixture is about travel, not frames"
    assert not tng._runaway(node), \
        "a 7 m displacement with 0.3 m of wheel travel is a jump, not a drive"


def test_no_tf_cannot_produce_a_runaway_either():
    """The pre-existing hole, closed by the same rule.

    With no map->base_link, world_xy() falls back to the raw odometry pose -- which
    in a long soak is metres from its origin while the robot sits at home. That
    fallback must never be measured against the room.
    """
    node = _Node(map_xy=(-5.73, 1.91), odom_xy=(-5.73, 1.91), offset=(5.91, -1.69),
                 moved=0.0, tf_ok=False)
    assert tng._pose_is_coherent(node) is None, "unjudgeable, not false"
    assert not tng._runaway(node)


def test_coherence_is_unjudgeable_rather_than_false_without_inputs():
    """None and False mean different things and only one blocks a verdict.

    A leg with no odom yet has nothing to corroborate; that is an absence of
    evidence, and it must not masquerade as evidence of incoherence.
    """
    node = _Node(**FALSE_RED)
    node.latest_odom = None
    assert tng._pose_is_coherent(node) is None
    node = _Node(**FALSE_RED)
    node._offset = None
    assert tng._pose_is_coherent(node) is None
