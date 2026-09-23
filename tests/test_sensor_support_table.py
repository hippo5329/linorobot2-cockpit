"""The documented sensor table must match the code, or it becomes folklore.

A table of "supported / DATA_RDY / bench" is exactly the kind of document that
rots: a driver gains an interrupt and the table still says no, someone wires a
pin on that word, and the board falls back to polling without anyone knowing
why. The DATA_RDY column is checkable against the source, so it is checked.

The `bench` column is NOT checkable here -- it records what answered on real
silicon, which no test can know. It is left to prose review, and the doc says
plainly what the word means (a bare-module reading, never a robot run).
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMU_H = os.path.join(ROOT, "firmware", "common", "lib", "imu", "default_imu.h")
FACTORY = os.path.join(ROOT, "firmware", "common", "lib", "sensor_factory",
                       "sensor_factory.cpp")
DOC = os.path.join(ROOT, "docs", "firmware.md")

# doc row label -> the class that implements it
ROW_TO_CLASS = {
    "MPU6050": "MPU6050IMU",
    "MPU9250": "MPU9250IMU",
    "ICM-42670-P": "ICM42670IMU",
    "ICM-20948": "ICM20948IMU",
    "QMI8658": "QMI8658IMU",
    "LSM6DSOX": "LSM6DSOXIMU",
    "GY85": "GY85IMU",
    "BNO085": "BNO085IMU",
}


def _src(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _has_drdy(cls_name):
    src = _src(IMU_H)
    start = src.index(f"class {cls_name}")
    nxt = src.find("\nclass ", start + 10)
    return "enableDataReadyInterrupt" in src[start:nxt if nxt != -1 else len(src)]


def _doc_rows():
    """The IMU table's rows: label -> the DATA_RDY cell."""
    doc = _src(DOC)
    table = doc[doc.index("| IMU | I2C addr | identified by | read on |"):]
    table = table[:table.index("\n\n")]
    rows = {}
    for line in table.splitlines()[2:]:
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 4:
            rows[cells[0]] = cells[3]      # the "read on" cell
    return rows


def test_every_imu_in_the_factory_has_a_row():
    """A driver the image carries and the table omits is a sensor nobody knows
    they can fit."""
    table = _src(FACTORY)
    names = re.findall(r'\{"(\w[\w-]*)",\s*makeIMU<', table)
    rows = " ".join(_doc_rows())
    for name in names:
        if name in ("fake", "mpu9150", "mpu6500"):
            continue                     # simulated, or an alias of a listed row
        key = {"icm42670": "ICM-42670", "icm20948": "ICM-20948",
               "lsm6dsox": "LSM6DSOX", "qmi8658": "QMI8658",
               "mpu6050": "MPU6050", "mpu9250": "MPU9250",
               "gy85": "GY85", "bno085": "BNO085"}[name]
        assert key in rows, f"{name} is in the sensor factory and not in the table"


def test_the_table_says_what_bench_means():
    """Without it, "bench" reads as "this robot works", and nothing in this
    project has run on a real robot yet."""
    doc = _src(DOC)
    assert "never that a robot drove" in doc or "not that a robot drove" in doc
    assert "October 2026" in doc


def test_no_driver_claims_a_data_ready_interrupt_any_more():
    """The interrupt path was removed on 2026-09-24. A driver that grows one
    back needs the table, the docs and a decision -- not a silent override."""
    src = _src(IMU_H)
    assert "enableDataReadyInterrupt" not in src, \
        "a driver implements a data-ready interrupt again; the path was removed"
    doc = _src(DOC)
    assert "Every part is read by polling." in doc
