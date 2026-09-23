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
    table = doc[doc.index("| IMU | I2C addr | identified by | DATA_RDY | read on | DRDY proven on |"):]
    table = table[:table.index("\n\n")]
    rows = {}
    for line in table.splitlines()[2:]:
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 4:
            rows[cells[0]] = cells[3]
    return rows


def test_the_data_ready_column_matches_the_drivers():
    rows = _doc_rows()
    for label, cell in rows.items():
        cls = next((c for k, c in ROW_TO_CLASS.items() if label.startswith(k)), None)
        assert cls, f"the table row {label!r} names no known driver class"
        documented = "no" not in cell.lower()
        assert documented == _has_drdy(cls), (
            f"{label}: the table says DATA_RDY {cell!r} but {cls} "
            f"{'has' if _has_drdy(cls) else 'has no'} enableDataReadyInterrupt()")


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


def test_reading_a_part_is_not_the_same_claim_as_proving_its_interrupt():
    """One column carrying both is how "bench: yes -- GenDrv" against a
    DATA_RDY "yes" was read as "the GenDrv's QMI8658 interrupt is wired". It
    is not: gendrv_config.yaml has no pins.imu.int and that chip is polled.
    """
    import glob
    import yaml
    doc = _src(DOC)
    assert "DRDY proven on" in doc, "the interrupt claim has been merged back into `read on`"
    # and the claim must match the configs: a board can only have proven a
    # DATA_RDY line if it ships a pin for one.
    wired = set()
    for path in glob.glob(os.path.join(ROOT, "config", "reference", "*_config.yaml")):
        cfg = yaml.safe_load(open(path, encoding="utf-8")) or {}
        pins = (cfg.get("base_controller") or {}).get("pins") or {}
        if (pins.get("imu") or {}).get("int", -1) not in (-1, None):
            wired.add(os.path.basename(path).replace("_config.yaml", ""))
    assert "yb_eet01" in wired, "the one board with a wired DATA_RDY lost its pin"
    assert "gendrv" not in wired, \
        "the GenDrv now has an IMU interrupt pin -- the table's QMI8658 row says it does not"
