"""map->odom carries the clock, not the scan's age.

slam_toolbox 2.10.0, publishTransformLoop:

    msg.header.stamp = restamp_tf_ ? now() + transform_timeout_
                                   : scan_timestamp + transform_timeout_;

and scan_header is assigned at the top of laserCallback, on every incoming
scan. With restamp_tf false the loop runs at 50 Hz but the STAMP only advances
as fast as /scan arrives, republishing the same one in between.

Every TF consumer is then hostage to scan latency. The bench Wi-Fi is strong
but shared, so a datagram queues behind other traffic and the scan lands late;
its stamp is old, map->odom inherits the age, and a controller asking for the
transform at "now" asks into the future. tf2 waits out the full tolerance --
which is why three separate transform_tolerance raises in this repo's history
bought nothing -- and then throws ExtrapolationException, which the controller
reports as error_code=102.

Measured: 601 ms of frozen stamp on a SERIAL leg, i.e. six scan periods with no
scan. Wi-Fi legs fail more often for the same reason with contention latency on
top (88% pass on jazzy vs 75% on lyrical was a different bug, now fixed, but
the transport gap remains).

Stamping with now() is what AMCL does for its own map->odom and is right for
the same reason: this is a slowly-varying CORRECTION, not an observation, so
its value is unchanged between scans and asserting it is current is true.
"""
import glob
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _slam(path):
    with open(path, encoding="utf-8") as fh:
        d = yaml.safe_load(fh)
    return ((d.get("slam") or {}).get("slam_toolbox") or {}).get("ros__parameters") or {}


def _configs():
    return sorted(glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")))


def test_every_config_stamps_the_correction_with_the_clock():
    for f in _configs():
        sl = _slam(f)
        if not sl:
            continue
        assert sl.get("restamp_tf") is True, (
            f"{os.path.basename(f)}: map->odom would carry the scan's age, so a late "
            f"scan starves every TF consumer")


def test_the_publish_period_is_still_faster_than_the_scan():
    """Restamping only helps if the loop actually runs between scans."""
    for f in _configs():
        sl = _slam(f)
        if not sl:
            continue
        assert 0 < sl["transform_publish_period"] <= 0.05, f


def test_a_dead_lidar_still_stops_the_robot():
    """Restamping must not turn a dead LiDAR into silent drift.

    It does not: the collision monitor's `scan` source times out and stops the
    robot, which is the component whose job that is. If that source were ever
    removed, restamping WOULD hide a dead sensor -- so the two are tested
    together, deliberately.
    """
    for f in _configs():
        with open(f, encoding="utf-8") as fh:
            d = yaml.safe_load(fh)
        cm = (d.get("nav2") or {}).get("collision_monitor", {}).get("ros__parameters")
        if not cm:
            continue
        assert "scan" in cm["observation_sources"], f
        assert cm["scan"]["enabled"] is True, f
        assert 0 < cm["source_timeout"] <= 2.0, (f, cm.get("source_timeout"))
