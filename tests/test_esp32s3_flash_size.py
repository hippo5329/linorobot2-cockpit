"""The ESP32-S3 image is built for the smallest S3 module (4 MB flash).

The bootloader trusts the image header's flash size. Built for the DevKitC-1's
8 MB, the same image put a 4 MB module (Yahboom YB-EET01) into a reboot loop
before setup() ever ran: `Detected size(4096k) smaller than the size in the
binary image header(8192k)`. 4 MB serves every module; the env partition is
the last sector of 4 MB by design.
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _section(name):
    with open(os.path.join(ROOT, "firmware", "common", "platformio_base.ini")) as fh:
        s = fh.read()
    m = re.search(rf"^\[{re.escape(name)}\]\n(.*?)(?=^\[|\Z)", s, re.S | re.M)
    assert m, name
    return m.group(1)


def test_s3_base_declares_4mb_flash():
    sec = _section("base_esp32s3")
    assert re.search(r"^board_upload\.flash_size\s*=\s*4MB$", sec, re.M), sec
    assert re.search(r"^board_upload\.maximum_size\s*=\s*4194304$", sec, re.M), sec


def test_the_partition_table_fits_in_4mb_with_the_env_sector_last():
    with open(os.path.join(ROOT, "firmware", "common", "partitions_lino.csv")) as fh:
        rows = [l.split(",") for l in fh if l.strip() and not l.startswith("#")]
    end = 0
    for r in rows:
        off, size = int(r[3].strip(), 0), int(r[4].strip(), 0)
        end = max(end, off + size)
    assert end <= 0x3FF000, f"partitions reach {end:#x}; the env sector is 0x3FF000-0x400000"
