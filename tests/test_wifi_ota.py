"""A Wi-Fi robot after it leaves the USB cable: found by name, asked by ping, written
over the air.

The fake board below speaks ArduinoOTA's side of the protocol the way the ESP32 core
does (libraries/ArduinoOTA: the password is kept as its MD5, the challenge is
md5(md5(password):nonce:cnonce), the image's MD5 is checked before "OK"), so the
uploader is held to the real responder, not to itself.
"""
import hashlib
import os
import socket
import stat
import threading

import pytest

import mcu_env
import ota_upload
import wifi_boards

BANNER = ("[fw] linorobot2_hardware app=base distro=jazzy built=2026-10-07T01:02:03Z "
          "git=abc1234 uid=A1B2C3D4E5F6 envota=1 envcrc=1A2B3C4D")


def _md5(b) -> str:
    return hashlib.md5(b if isinstance(b, bytes) else b.encode()).hexdigest()


class FakeOtaBoard:
    """One OTA session at a time, on 127.0.0.1."""

    def __init__(self, password="", accept=(0, 100), corrupt=False, ack_every=3):
        self.password_md5 = _md5(password) if password else ""
        self.accept, self.corrupt, self.ack_every = accept, corrupt, ack_every
        self.received = None
        self.kind = None
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(5)
        self.port = self.udp.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        try:
            msg, peer = self.udp.recvfrom(256)
        except socket.timeout:
            return
        cmd, tcp_port, size, md5 = msg.decode().split()
        cmd, tcp_port, size = int(cmd), int(tcp_port), int(size)
        if cmd not in self.accept:
            self.udp.sendto(b"ERR: no partition", peer)
            return
        if self.password_md5:
            nonce = _md5("nonce")
            self.udp.sendto(f"AUTH {nonce}".encode(), peer)
            auth, peer = self.udp.recvfrom(256)
            _, cnonce, response = auth.decode().split()
            if response != _md5(f"{self.password_md5}:{nonce}:{cnonce}"):
                self.udp.sendto(b"Authentication Failed", peer)
                return
        self.udp.sendto(b"OK", peer)
        self.kind = cmd
        conn = socket.create_connection(("127.0.0.1", tcp_port), timeout=5)
        data = b""
        n = 0
        while len(data) < size:
            chunk = conn.recv(4096)
            if not chunk:
                break
            data += chunk
            n += 1
            if n % self.ack_every == 0 or len(data) >= size:     # coalesced counts
                conn.sendall(str(len(data)).encode())
        ok = _md5(data) == md5 and not self.corrupt
        conn.sendall(b"OK" if ok else b"ERROR[4]: MD5 Check Failed")
        conn.close()
        self.received = data if ok else None


def test_an_image_is_written_and_confirmed():
    board = FakeOtaBoard()
    image = os.urandom(50_000)
    ota_upload.upload("127.0.0.1", image, "app", board.port, timeout=5)
    board.thread.join(5)
    assert board.received == image and board.kind == 0


def test_the_env_goes_by_the_filesystem_command():
    board = FakeOtaBoard()
    block = os.urandom(4096)
    ota_upload.upload("127.0.0.1", block, "env", board.port, timeout=5)
    board.thread.join(5)
    assert board.received == block and board.kind == 100


def test_the_password_answers_the_boards_challenge():
    board = FakeOtaBoard(password="s3cret")
    ota_upload.upload("127.0.0.1", b"x" * 3000, "app", board.port, password="s3cret", timeout=5)
    board.thread.join(5)
    assert board.received == b"x" * 3000


def test_a_wrong_password_writes_nothing():
    board = FakeOtaBoard(password="s3cret")
    with pytest.raises(ota_upload.OtaError, match="password"):
        ota_upload.upload("127.0.0.1", b"x" * 3000, "app", board.port, password="nope", timeout=3)
    assert board.received is None


def test_no_password_for_a_board_that_wants_one_is_said_plainly():
    board = FakeOtaBoard(password="s3cret")
    with pytest.raises(ota_upload.OtaError, match="asks for an OTA password"):
        ota_upload.upload("127.0.0.1", b"x", "app", board.port, timeout=3)


def test_a_board_that_rejects_the_image_is_an_error():
    board = FakeOtaBoard(corrupt=True)
    with pytest.raises(ota_upload.OtaError, match="did not confirm"):
        ota_upload.upload("127.0.0.1", b"y" * 9000, "app", board.port, timeout=5)


def test_an_old_board_refusing_the_env_points_at_usb():
    board = FakeOtaBoard(accept=(0,))
    with pytest.raises(ota_upload.OtaError, match="flash it once over USB"):
        ota_upload.upload("127.0.0.1", b"z" * 4096, "env", board.port, timeout=3)


def test_the_auth_response_is_the_cores_digest():
    cnonce, resp = ota_upload.auth_response("pw", "N", "f", 3, "m", "h")
    assert resp == _md5(f"{_md5('pw')}:N:{cnonce}") and len(cnonce) == 32


# ---------------------------------------------------------------- wifi_boards

@pytest.fixture
def cfg(tmp_path, monkeypatch):
    monkeypatch.setenv("COCKPIT_CONFIG_DIR", str(tmp_path))
    return tmp_path


def test_a_banner_names_the_board_its_build_and_its_env(cfg):
    b = wifi_boards.parse("<133>1 - linorobot2 hardware - - - ﻿" + BANNER)
    assert b == {"app": "base", "distro": "jazzy", "git": "abc1234",
                 "built": "2026-10-07T01:02:03Z", "uid": "A1B2C3D4E5F6",
                 "envota": True, "envcrc": "1A2B3C4D"}
    assert wifi_boards.parse("[fw] linorobot2_hardware app=base built=x git=y flashid=AB") == {}
    assert wifi_boards.parse("agent available 1234") == {}


def test_a_robot_is_found_by_name_through_its_uid(cfg):
    wifi_boards.record_banner(BANNER, "192.0.2.77")
    wifi_boards.remember_robot("yb_eet01", "a1b2c3d4e5f6", "esp32s3")
    assert wifi_boards.robot_uid("yb_eet01") == "A1B2C3D4E5F6"
    assert wifi_boards.board("A1B2C3D4E5F6")["ip"] == "192.0.2.77"
    assert set(wifi_boards.heard_boards()) == {"A1B2C3D4E5F6"}
    assert wifi_boards.find_robot_board("never_flashed") == ("", {})


def test_ping_asks_the_board_and_records_its_answer(cfg):
    resp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    resp.bind(("127.0.0.1", 0))

    def answer():
        resp.settimeout(3)
        msg, peer = resp.recvfrom(64)
        if msg.startswith(b"lino?"):
            resp.sendto(BANNER.encode(), peer)
    t = threading.Thread(target=answer, daemon=True)
    t.start()
    got = wifi_boards.ping("127.0.0.1", resp.getsockname()[1], timeout=2)
    t.join(3)
    resp.close()
    assert set(got) == {"A1B2C3D4E5F6"} and got["A1B2C3D4E5F6"]["ip"] == "127.0.0.1"


# ---------------------------------------------------------------- OTA password

def _robot(tmp_path, name, transport):
    p = tmp_path / f"{name}_config.yaml"
    p.write_text(f"robot:\n  name: {name}\nbase_controller:\n  name: esp32\n  transport: {transport}\n")
    return str(p)


def test_a_wifi_robot_gets_its_own_password_once(cfg):
    path = _robot(cfg, "wifibot", "udp4")
    pw = mcu_env.ensure_ota_password(path, str(cfg / "secrets.yaml"))
    assert len(pw) >= 20
    assert mcu_env.ensure_ota_password(path, str(cfg / "secrets.yaml")) == pw
    f = cfg / "secrets.ota.yaml"
    assert stat.S_IMODE(os.stat(f).st_mode) == 0o600
    assert pw not in (cfg / "wifibot_config.yaml").read_text()


def test_a_serial_robot_gets_no_password(cfg):
    assert mcu_env.ensure_ota_password(_robot(cfg, "serialbot", "serial")) == ""
    assert not (cfg / "secrets.ota.yaml").exists()


def test_the_robots_own_password_wins(cfg):
    path = _robot(cfg, "wifibot", "udp4")
    (cfg / "secrets.ota.yaml").write_text("robots:\n  wifibot:\n    ota_password: own\n")
    params = mcu_env.load_yaml(path)
    assert mcu_env.ota_password_for(params, path, {"telemetry": {"ota_password": "shared"}}) == "own"
    assert mcu_env.ota_password_for({"robot": {"name": "other"}}, None,
                                    {"telemetry": {"ota_password": "shared"}}) == "shared"


# ---------------------------------------------------------------- names and addresses

def test_the_robot_name_becomes_a_dns_label():
    assert mcu_env.mdns_hostname("yb_eet01") == "yb-eet01"
    assert mcu_env.mdns_hostname("Bare ESP32S3 cdc!") == "bare-esp32s3-cdc"
    assert len(mcu_env.mdns_hostname("x" * 100)) == 63


def test_every_robot_env_carries_its_hostname(cfg):
    path = _robot(cfg, "yb_eet01", "udp4")
    env = mcu_env.env_from_config(path, str(cfg / "secrets.yaml"), default_host="192.0.2.9")
    assert env["hostname"] == "yb-eet01"


def _mdns_answer(host: str, ip: str) -> bytes:
    """A response the way responders write it: the question echoed, the answer's
    name a compression pointer back to it."""
    import struct
    head = struct.pack(">HHHHHH", 0, 0x8400, 1, 1, 0, 0)
    q = wifi_boards._dns_name(host) + struct.pack(">HH", 1, 1)
    ans = b"\xc0\x0c" + struct.pack(">HHIH", 1, 0x8001, 120, 4) + socket.inet_aton(ip)
    return head + q + ans


def test_an_mdns_answer_is_read_through_name_compression():
    pkt = _mdns_answer("yb-eet01.local", "192.0.2.77")
    assert wifi_boards.parse_mdns_a(pkt, "yb-eet01.local") == "192.0.2.77"
    assert wifi_boards.parse_mdns_a(pkt, "other.local") == ""
    assert wifi_boards.parse_mdns_a(b"\x00" * 5, "yb-eet01.local") == ""


def test_the_robot_is_remembered_with_the_address_its_usb_banner_named(cfg):
    wifi_boards.remember_robot("yb_eet01", "A1B2C3D4E5F6", "esp32s3", "192.0.2.10")
    assert wifi_boards.robot_ip("yb_eet01") == "192.0.2.10"
    wifi_boards.record_banner(BANNER, "192.0.2.77")      # it moved; its banner says where
    assert wifi_boards.robot_ip("yb_eet01") == "192.0.2.77"


def test_the_network_name_is_the_users_hostname_else_the_robots_name():
    p = {"robot": {"name": "yb_eet01"}, "base_controller": {"telemetry": {}}}
    assert mcu_env.robot_mdns_name(p) == "yb_eet01"
    p["base_controller"]["telemetry"]["hostname"] = "Kitchen_Bot"
    assert mcu_env.robot_mdns_name(p) == "Kitchen_Bot"
    assert mcu_env.mdns_hostname(mcu_env.robot_mdns_name(p)) == "kitchen-bot"
    assert mcu_env.robot_ip_for(p) == ""
    p["base_controller"]["robot_ip"] = "192.0.2.50"
    assert mcu_env.robot_ip_for(p) == "192.0.2.50"
