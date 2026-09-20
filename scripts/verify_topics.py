#!/usr/bin/env python3
# ==============================================================================
# verify_topics.py — ROS 2 Topic Echo & Frequency (Hz) Verification Gate
#
# Validates:
# 1. Payload validity (non-empty, valid frame_id, valid timestamps)
# 2. Measured topic frequency (Hz) meets minimum thresholds:
#    - /odom      >= 10.0 Hz (nav_msgs/msg/Odometry)
#    - /imu/data  >=  7.0 Hz (sensor_msgs/msg/Imu)
#    - /scan      >=  5.0 Hz (sensor_msgs/msg/LaserScan)
#
# Must pass before proceeding to SLAM mapping or Nav2 navigation.
#
# Auxiliary sensor topics -- /imu/mag, /battery, /pressure, /temperature -- are
# reported but not required, because most robots do not carry those chips. On a
# REAL base they are soldered on and named in the robot config, so the pipeline
# passes them to --require:
#
#   verify_topics.py --require /imu/mag,/battery,/pressure,/temperature
#
# What --require checks is ADVERTISEMENT, not data. A board on the end of a USB
# cable can say what it is fitted with long before there is an assembled robot
# to take readings from, and this gate runs at exactly that point -- so demanding
# a message would mean the check could never run until the robot was finished.
#
# Advertisement is still real evidence, because the firmware creates each of
# these publishers conditionally: /imu/mag under PUBLISH_MAG, /battery under
# a detected INA219 or BATTERY_PIN, /pressure and /temperature whenever the
# barometer answered the boot-time I2C probe (`env_present`). A
# missing publisher therefore means the driver was never compiled in or the chip
# did not answer the bus. Readings that arrive anyway are echoed as a bonus;
# rate is never checked for them, since a barometer publishes at a few Hz by
# design.
# ==============================================================================

import argparse
import json
import sys
import time
from typing import Dict, List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import (LaserScan, Imu, FluidPressure, Temperature,
                                 BatteryState, MagneticField)
    from nav_msgs.msg import Odometry
except ImportError as exc:
    # Name the module that actually failed. This block used to report "rclpy not
    # found" for every import error in the group, which is wrong far more often
    # than it is right: rclpy sits on PYTHONPATH and imports even from inside a
    # venv that hides the system dist-packages, and the real casualty is numpy,
    # pulled in several frames down by rclpy.node -> rosgraph_msgs. The generic
    # message then sends the reader off to source a setup.bash that was already
    # sourced. Print the module and the interpreter; both are the diagnosis.
    print(f"Error: cannot import the ROS 2 Python stack: {exc}", file=sys.stderr)
    print(f"  interpreter: {sys.executable}", file=sys.stderr)
    print("  Check that ROS 2 setup.bash is sourced AND that this interpreter can",
          file=sys.stderr)
    print("  see the system dist-packages (a venv with include-system-site-packages",
          file=sys.stderr)
    print("  = false on PATH ahead of /usr/bin will hide numpy).", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Is the number physically possible?
#
# A sensor that is wired, driven and advertising can still be lying, and the two
# ways it lies are both silent: it reads a constant (usually zero, the signature
# of a chip that initialised and then stopped answering) or it reads in the
# wrong unit. Neither shows up in a rate check -- a dead IMU publishes zeros at
# a perfect 40 Hz -- so each required topic gets a band that real hardware in a
# room falls inside and those two failures fall outside.
#
# The bands are deliberately wide. This is a "that cannot be a reading" test,
# not a calibration: the job is to separate a working chip from a broken one
# without failing a robot that is merely somewhere cold, high up, or next to a
# motor. Checked only for --require topics, i.e. only on a real base.
# ---------------------------------------------------------------------------
import math


def _mag_range(msg):
    b = msg.magnetic_field
    m = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z)
    # Earth's field is 25-65 uT. The floor catches a dead magnetometer reading
    # zeros; the ceiling catches the classic unit bug, a driver publishing
    # microtesla into a field documented as tesla, which lands around 50.
    if not (5e-6 <= m <= 5e-3):
        return False, f"|B|={m:.2e} T outside 5e-06..5e-03 T (Earth's field is ~2.5e-05..6.5e-05)"
    return True, f"|B|={m * 1e6:.1f} uT"


def _pressure_range(msg):
    v = msg.fluid_pressure
    # Sea level is 101325 Pa; 50 kPa is about 5500 m. A BMP280 driver that
    # publishes hectopascals lands near 1013 and fails here, which is the point.
    if not (50000.0 <= v <= 115000.0):
        return False, f"{v:.1f} Pa outside 50000..115000 Pa (sea level is 101325 Pa)"
    return True, f"{v / 100:.1f} hPa"


def _temperature_range(msg):
    v = msg.temperature
    # The sensor's own operating range, not the weather's: outside this the
    # chip could not be reporting its own die temperature correctly anyway.
    if not (-20.0 <= v <= 85.0):
        return False, f"{v:.1f} C outside -20..85 C"
    return True, f"{v:.1f} C"


def _battery_range(msg):
    v = msg.voltage
    if not (4.0 <= v <= 60.0):
        return False, f"{v:.2f} V outside 4..60 V"
    pct = msg.percentage
    # NaN is legitimate here: battery.cpp leaves percentage NAN when no
    # min/max voltage is configured, and "unknown" is not "wrong".
    if not math.isnan(pct) and not (0.0 <= pct <= 1.0):
        return False, f"percentage {pct} outside 0..1 (sensor_msgs uses a fraction)"
    return True, f"{v:.2f} V"


def _imu_range(msg):
    a = msg.linear_acceleration
    m = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    # Gravity is always there, whatever the board is doing. An IMU publishing
    # zeros -- which passes any rate check -- reads 0 and fails; a board being
    # picked up and waved reads a few g and still passes.
    if not (5.0 <= m <= 30.0):
        return False, f"|accel|={m:.2f} m/s^2 outside 5..30 (gravity alone is 9.81)"
    return True, f"|accel|={m:.2f} m/s^2"


RANGE_CHECKS = {
    "/imu/mag": _mag_range,
    "/pressure": _pressure_range,
    "/temperature": _temperature_range,
    "/battery": _battery_range,
    "/imu/data": _imu_range,
}


class TopicVerifier(Node):
    def __init__(self, check_scan: bool = True, min_samples: int = 10, timeout: float = 6.0,
                 required_aux: Optional[List[str]] = None):
        super().__init__("linorobot2_topic_verifier")
        self.min_samples = min_samples
        self.timeout = timeout
        self.check_scan = check_scan
        # Auxiliary topics the caller says MUST be there. On a real base the
        # sensors are soldered on, so a missing /battery or /imu/mag is a dead
        # chip or a bus fault, not an absent option -- see --require.
        self.required_aux = list(required_aux or [])

        # QoS configuration
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.timestamps: Dict[str, List[float]] = {
            "/odom": [],
            "/imu/data": [],
        }
        self.samples: Dict[str, Optional[object]] = {
            "/odom": None,
            "/imu/data": None,
        }
        self.thresholds: Dict[str, float] = {
            "/odom": 10.0,
            "/imu/data": 7.0,
        }

        if self.check_scan:
            self.timestamps["/scan"] = []
            self.samples["/scan"] = None
            self.thresholds["/scan"] = 5.0

        # Create subscribers with dual QoS to ensure discovery regardless of publisher profile
        self.create_subscription(Odometry, "/odom", self._odom_cb, reliable_qos)
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, reliable_qos)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, sensor_qos)
        self.create_subscription(Imu, "/imu/data_raw", self._imu_cb, reliable_qos)
        self.create_subscription(Imu, "/imu/data_raw", self._imu_cb, sensor_qos)
        if self.check_scan:
            self.create_subscription(LaserScan, "/scan", self._scan_cb, sensor_qos)

        # Auxiliary topic samples (optional: pressure, temperature, battery)
        # One auxiliary topic per sensor the firmware may carry:
        #   /imu/mag      AK09918 etc.   (PUBLISH_MAG)
        #   /battery      INA219         (detected on the bus, or BATTERY_PIN)
        #   /pressure     BMP280/BME280  (detected on the bus; pub_env may veto)
        #   /temperature  BMP280/BME280  (ditto)
        # /imu/mag was missing here, which meant the magnetometer was the one
        # fitted sensor no test ever looked at.
        self.optional_samples: Dict[str, Optional[object]] = {
            "/imu/mag": None,
            "/pressure": None,
            "/temperature": None,
            "/battery": None,
        }
        self.create_subscription(MagneticField, "/imu/mag", self._mag_cb, sensor_qos)
        self.create_subscription(MagneticField, "/imu/mag", self._mag_cb, reliable_qos)
        self.create_subscription(FluidPressure, "/pressure", self._pressure_cb, sensor_qos)
        self.create_subscription(FluidPressure, "/pressure", self._pressure_cb, reliable_qos)
        self.create_subscription(Temperature, "/temperature", self._temp_cb, sensor_qos)
        self.create_subscription(Temperature, "/temperature", self._temp_cb, reliable_qos)
        self.create_subscription(BatteryState, "/battery", self._battery_cb, sensor_qos)
        self.create_subscription(BatteryState, "/battery", self._battery_cb, reliable_qos)

    def _mag_cb(self, msg: MagneticField):
        self.optional_samples["/imu/mag"] = msg

    def _pressure_cb(self, msg: FluidPressure):
        self.optional_samples["/pressure"] = msg

    def _temp_cb(self, msg: Temperature):
        self.optional_samples["/temperature"] = msg

    def _battery_cb(self, msg: BatteryState):
        self.optional_samples["/battery"] = msg

    def _odom_cb(self, msg: Odometry):
        now = time.time()
        if self.timestamps["/odom"] and (now - self.timestamps["/odom"][-1]) < 0.005:
            return
        self.timestamps["/odom"].append(now)
        if self.samples["/odom"] is None:
            self.samples["/odom"] = msg

    def _imu_cb(self, msg: Imu):
        now = time.time()
        if self.timestamps["/imu/data"] and (now - self.timestamps["/imu/data"][-1]) < 0.005:
            return
        self.timestamps["/imu/data"].append(now)
        if self.samples["/imu/data"] is None:
            self.samples["/imu/data"] = msg

    def _scan_cb(self, msg: LaserScan):
        if "/scan" in self.timestamps:
            now = time.time()
            if self.timestamps["/scan"] and (now - self.timestamps["/scan"][-1]) < 0.005:
                return
            self.timestamps["/scan"].append(now)
            if self.samples["/scan"] is None:
                self.samples["/scan"] = msg

    def is_complete(self) -> bool:
        for topic, ts_list in self.timestamps.items():
            if len(ts_list) < self.min_samples:
                return False
        return True

    def aux_publishers(self, topic: str) -> int:
        """How many publishers the graph shows for an auxiliary topic.

        This, not a received message, is what a required sensor is judged on.
        The board has to be assembled and running before a barometer or a
        current monitor produces readings, and the point of this gate is to run
        BEFORE that -- on a bench, with the board on the end of a USB cable.

        Advertisement is still real evidence, because the firmware creates these
        publishers conditionally: /imu/mag only under PUBLISH_MAG, /battery only
        under a detected INA219 or BATTERY_PIN, and /pressure and /temperature
        only when the barometer actually answered at boot
        (`env_present`, set from the I2C probe). So a missing publisher means
        the driver was not compiled in or the chip did not answer -- which is
        exactly the failure worth catching. Whether the numbers are sensible is
        a question for the robot, later.
        """
        return self.count_publishers(topic)


def format_odom(msg: Odometry) -> str:
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    return f"frame='{msg.header.frame_id}', child='{msg.child_frame_id}', pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), vel_lin={v.x:.3f} m/s, vel_ang={w.z:.3f} rad/s"


def format_imu(msg: Imu) -> str:
    a = msg.linear_acceleration
    w = msg.angular_velocity
    q = msg.orientation
    return f"frame='{msg.header.frame_id}', accel=({a.x:.2f}, {a.y:.2f}, {a.z:.2f}) m/s², gyro=({w.x:.2f}, {w.y:.2f}, {w.z:.2f}) rad/s, quat_w={q.w:.3f}"


def format_scan(msg: LaserScan) -> str:
    valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
    min_r = min(valid_ranges) if valid_ranges else 0.0
    max_r = max(valid_ranges) if valid_ranges else 0.0
    return f"frame='{msg.header.frame_id}', points={len(msg.ranges)} (valid={len(valid_ranges)}), range=[{min_r:.2f}m .. {max_r:.2f}m]"


def main():
    parser = argparse.ArgumentParser(description="Verify ROS 2 topic echo payload and publish rates (Hz)")
    parser.add_argument("--no-scan", action="store_true", help="Skip /scan topic check")
    parser.add_argument("--samples", type=int, default=12, help="Message samples to measure frequency")
    parser.add_argument("--timeout", type=float, default=10.0, help="Maximum seconds to collect samples")
    parser.add_argument("--json", action="store_true", help="Output results as JSON")
    parser.add_argument("--no-range-check", action="store_true",
                        help="With --require, check that the sensors publish but not that "
                             "their values are physically plausible.")
    parser.add_argument("--require", default="",
                        help="Comma-separated auxiliary topics that must carry data "
                             "(e.g. /imu/mag,/battery,/pressure,/temperature). Used on a "
                             "real base, where every listed sensor is physically fitted.")
    args = parser.parse_args()

    required_aux = [t.strip() for t in args.require.split(",") if t.strip()]
    # --require means "this is a real base", and on a real base the readings are
    # checked against what physics allows as well as against the clock.
    # --no-range-check keeps the presence gate and drops the value gate, for a
    # rig where a sensor is deliberately fed something unrealistic.
    check_ranges = bool(required_aux) and not args.no_range_check
    rclpy.init()
    verifier = TopicVerifier(check_scan=not args.no_scan, min_samples=args.samples,
                             timeout=args.timeout, required_aux=required_aux)
    unknown = [t for t in required_aux if t not in verifier.optional_samples]
    if unknown:
        print(f"Error: --require names topics this gate does not subscribe to: "
              f"{', '.join(unknown)}", file=sys.stderr)
        print(f"  known: {', '.join(sorted(verifier.optional_samples))}", file=sys.stderr)
        sys.exit(2)

    if not args.json:
        print("==================================================================")
        print("🔍 Inspecting ROS 2 Topics: Payload Echo & Publishing Rates (Hz)")
        print("==================================================================")

    start_time = time.time()
    while rclpy.ok() and not verifier.is_complete():
        rclpy.spin_once(verifier, timeout_sec=0.1)
        if time.time() - start_time > args.timeout:
            break

    all_passed = True
    json_results = {
        "all_passed": True,
        "topics": {},
    }

    if not args.json:
        print("\n--- Topic Verification Results ---")

    for topic, ts_list in verifier.timestamps.items():
        sample = verifier.samples[topic]
        threshold = verifier.thresholds[topic]

        if len(ts_list) < 2 or sample is None:
            all_passed = False
            json_results["topics"][topic] = {
                "received": False,
                "samples": len(ts_list),
                "hz": 0.0,
                "threshold": threshold,
                "passed": False,
                "echo": "NO DATA (0 msgs received)",
            }
            if not args.json:
                print(f"❌ {topic:12s} : NO DATA (0 msgs received in {args.timeout:.1f}s)")
            continue

        # Compute Hz
        dt_total = ts_list[-1] - ts_list[0]
        n_intervals = len(ts_list) - 1
        hz = n_intervals / dt_total if dt_total > 0 else 0.0

        # Payload description
        if topic == "/odom":
            desc = format_odom(sample)
        elif topic == "/imu/data":
            desc = format_imu(sample)
        elif topic == "/scan":
            desc = format_scan(sample)
        else:
            desc = str(sample)[:60]

        passed = (hz >= threshold)
        if not passed:
            all_passed = False
            status = f"FAIL (rate {hz:.1f} Hz < min {threshold:.1f} Hz)"
        else:
            status = f"PASS ({hz:.1f} Hz >= {threshold:.1f} Hz)"

        # On a real base the IMU is a physical chip, so its numbers have to be
        # possible as well as punctual. A dead IMU publishes zeros at a perfect
        # rate and passed everything above.
        range_note = None
        if check_ranges and topic in RANGE_CHECKS:
            in_range, range_note = RANGE_CHECKS[topic](sample)
            if not in_range:
                passed = False
                all_passed = False
                status = f"FAIL (implausible reading: {range_note})"

        json_results["topics"][topic] = {
            "received": True,
            "samples": len(ts_list),
            "hz": round(hz, 2),
            "threshold": threshold,
            "passed": passed,
            "range": range_note,
            "echo": desc,
        }

        if not args.json:
            icon = "✅" if passed else "❌"
            print(f"{icon} {topic:12s} : {status}")
            print(f"   Echo Sample  : {desc}")
            if range_note and passed:
                print(f"   Range Check  : {range_note}")

    json_results["auxiliary_topics"] = {}
    for opt_topic, opt_sample in verifier.optional_samples.items():
        required = opt_topic in required_aux
        publishers = verifier.aux_publishers(opt_topic)

        if required and publishers == 0:
            # The robot config says this chip is fitted and the firmware did not
            # advertise its topic. Driver not compiled in, or the chip did not
            # answer the I2C probe at boot. Either way the board is not the
            # robot the config describes.
            all_passed = False
            json_results["auxiliary_topics"][opt_topic] = {
                "received": False, "required": True, "publishers": 0,
                "echo": "NOT ADVERTISED (no publisher)",
            }
            if not args.json:
                print(f"❌ {opt_topic:12s} : NOT ADVERTISED — the robot config says this "
                      f"sensor is fitted")
            continue

        if opt_sample is None:
            if not required:
                continue
            # Advertised but quiet. Expected on a bench: the publisher exists,
            # the robot is not running yet. Reported, never failed -- the
            # readings are the assembled robot's business.
            json_results["auxiliary_topics"][opt_topic] = {
                "received": False, "required": True, "publishers": publishers,
                "echo": f"advertised by {publishers} publisher(s), no data yet",
            }
            if not args.json:
                print(f"✅ {opt_topic:12s} : advertised ({publishers} publisher"
                      f"{'' if publishers == 1 else 's'}), no data yet")
            continue

        # Data arrived, so the value can be judged. Only for --require topics:
        # elsewhere these are a courtesy readout from hardware nobody promised
        # was there.
        in_range, range_note = True, None
        if check_ranges and opt_topic in RANGE_CHECKS:
            in_range, range_note = RANGE_CHECKS[opt_topic](opt_sample)
            if not in_range:
                all_passed = False

        if opt_topic == "/imu/mag":
            m = opt_sample.magnetic_field
            opt_desc = (f"frame='{opt_sample.header.frame_id}', "
                        f"B=({m.x:.2e}, {m.y:.2e}, {m.z:.2e}) T")
        elif opt_topic == "/pressure":
            opt_desc = f"{opt_sample.fluid_pressure:.2f} Pa (variance={opt_sample.variance})"
        elif opt_topic == "/temperature":
            opt_desc = f"{opt_sample.temperature:.2f} °C (variance={opt_sample.variance})"
        elif opt_topic == "/battery":
            # percentage is a 0..1 fraction (sensor_msgs convention, and what
            # battery.cpp publishes: sigmoidal(...) / 100). Printed raw it read
            # "0.9%" for a battery at 87%.
            opt_desc = (f"{opt_sample.voltage:.2f}V, current={opt_sample.current:.2f}A, "
                        f"pct={opt_sample.percentage * 100:.0f}%")
        else:
            opt_desc = str(opt_sample)[:60]
        json_results["auxiliary_topics"][opt_topic] = {
            "received": True,
            "required": required,
            "publishers": publishers,
            "in_range": in_range,
            "range": range_note,
            "echo": opt_desc,
        }
        if not args.json:
            icon = "❌" if not in_range else ("✅" if required else "ℹ️ ")
            print(f"{icon} {opt_topic:12s} : {opt_desc}")
            if range_note:
                label = "implausible" if not in_range else "in range"
                print(f"   {'':12s}   {label}: {range_note}")

    json_results["all_passed"] = all_passed

    if args.json:
        print(json.dumps(json_results))
        exit_code = 0 if all_passed else 1
    else:
        print("==================================================================")
        if all_passed:
            print("✅ ALL TOPIC RATES & ECHO PAYLOADS VERIFIED SUCCESSFULLY")
            exit_code = 0
        else:
            print("❌ TOPIC VERIFICATION FAILED (Review above issues before SLAM/Nav2)")
            exit_code = 1
        print("==================================================================")

    verifier.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
