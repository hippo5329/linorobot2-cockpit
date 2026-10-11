"""The UNO Q's STM32 boot lines, set by the cockpit instead of Arduino's router.

arduino-router.service drives gpiochip1 line 37 low (BOOT0) before it starts and line 70
high after. The cockpit runs with the router off (it holds /dev/ttyHS1), so after a power
cycle nothing set BOOT0 and the STM32 booted its ROM bootloader. The 1-Click now sets the
lines before every SWD flash, and docker/unoq/lino-stm32-boot.service sets them at boot.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import unoq_swd  # noqa: E402


def test_the_lines_are_the_routers():
    assert unoq_swd.BOOT_CHIP == "/dev/gpiochip1"
    assert unoq_swd.BOOT_LINES == ((37, 0), (70, 1))


def test_the_request_matches_the_kernel_uapi():
    req, size = unoq_swd.line_request(unoq_swd.BOOT_LINES)
    assert size == 592                                   # sizeof(struct gpio_v2_line_request)
    assert unoq_swd._gpio_v2_get_line_ioctl(size) == 0xC250B407   # GPIO_V2_GET_LINE_IOCTL
    assert list(req.offsets[:2]) == [37, 70] and req.num_lines == 2
    assert req.config.flags == unoq_swd._GPIO_V2_LINE_FLAG_OUTPUT
    att = req.config.attrs[0]
    assert req.config.num_attrs == 1 and att.attr.id == unoq_swd._GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES
    assert att.mask == 0b11 and att.attr.values == 0b10    # 37 low, 70 high


def test_set_boot_lines_requests_then_releases(monkeypatch):
    calls = []
    monkeypatch.setattr(unoq_swd.os, "open", lambda p, f: calls.append(("open", p)) or 7)
    monkeypatch.setattr(unoq_swd.os, "close", lambda fd: calls.append(("close", fd)))
    import fcntl

    def fake_ioctl(fd, num, req):
        calls.append(("ioctl", fd, num))
        req.fd = 9
    monkeypatch.setattr(fcntl, "ioctl", fake_ioctl)
    unoq_swd.set_boot_lines()
    assert calls[0] == ("open", "/dev/gpiochip1")
    assert calls[1] == ("ioctl", 7, 0xC250B407)
    assert ("close", 9) in calls and ("close", 7) in calls


def test_a_chip_it_cannot_open_is_a_clear_error(monkeypatch):
    def nope(p, f):
        raise PermissionError(13, "Permission denied")
    monkeypatch.setattr(unoq_swd.os, "open", nope)
    try:
        unoq_swd.set_boot_lines()
    except unoq_swd.SwdError as exc:
        assert "gpiochip1" in str(exc) and "GPIO chips" in str(exc)
    else:
        raise AssertionError("no SwdError")


def test_the_1click_sets_them_before_the_flash():
    src = open(os.path.join(ROOT, "scripts", "one_click_pipeline.py")).read()
    body = src[src.index("def flash_unoq("):src.index("def flash_firmware(")]
    assert body.index("unoq_swd.set_boot_lines()") < body.index("unoq_swd.flash(env_bin, elf)")


def test_the_boot_service_ships_and_the_compose_file_says_to_install_it():
    svc = open(os.path.join(ROOT, "docker", "unoq", "lino-stm32-boot.service")).read()
    assert "gpioset -c /dev/gpiochip1 -t0 37=0" in svc and "gpioset -c /dev/gpiochip1 -t0 70=1" in svc
    assert "WantedBy=multi-user.target" in svc
    compose = open(os.path.join(ROOT, "docker-compose.unoq.yml")).read()
    assert "lino-stm32-boot.service" in compose and "enable --now lino-stm32-boot" in compose
