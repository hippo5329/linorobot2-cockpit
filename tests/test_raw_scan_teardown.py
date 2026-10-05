"""raw_scan's message borrows the firmware's scan batch; teardown must not free it.

flushRawScan() points raw_scan_msg.data.data at raw_scan_batch, which initRawScan()
allocates once and the LiDAR (simulated or real) keeps filling. destroyEntities()
runs std_msgs__msg__UInt8MultiArray__fini(&raw_scan_msg), and __fini frees data.data.
Without detaching the buffer first, every agent reconnect freed the batch while the
scan still wrote into it -- heap corruption, found as bus faults inside the allocator
on the Arduino UNO Q (2026-10-05).
"""
import os
import re

MAIN = os.path.join(os.path.dirname(__file__), "..", "firmware", "src", "main.cpp")


def test_raw_scan_buffer_is_detached_before_fini():
    src = open(MAIN, encoding="utf-8").read()
    fini = src.index("std_msgs__msg__UInt8MultiArray__fini(&raw_scan_msg)")
    block = src[src.rindex("if (raw_scan_pub_ready)", 0, fini):fini]
    assert re.search(r"raw_scan_msg\.data\.data\s*=\s*NULL", block), (
        "destroyEntities() must set raw_scan_msg.data.data = NULL before __fini: "
        "the buffer is raw_scan_batch, owned by the firmware, not the message")


def test_the_batch_is_what_the_message_borrows():
    """The premise of the test above: if this changes, revisit the teardown."""
    src = open(MAIN, encoding="utf-8").read()
    assert "raw_scan_msg.data.data = raw_scan_batch;" in src
