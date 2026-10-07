#!/usr/bin/env python3
"""Send an image to an ESP32/ESP32-S3 over the air (the ArduinoOTA protocol).

A Wi-Fi robot leaves the computer after its first USB flash; from then on this is
how it is written. Two kinds of write, both a whole image with its MD5 checked by
the board before it reboots:

    app   the application (firmware.bin) -> the next OTA slot      ArduinoOTA U_FLASH
    env   the 4 KB env block (mcu_env.py build) -> the `env` partition  U_SPIFFS

The env write needs the board's partition table to carry `env` as a DATA/SPIFFS
partition and an image that points ArduinoOTA's filesystem command at it
(firmware/common/lib/wifi/ota.cpp; the banner says `envota=1`). An older board
answers the invitation with an error and nothing is written.

The protocol, as ArduinoOTA's responder speaks it:

  1. UDP to the board's OTA port:  "<cmd> <our tcp port> <size> <md5>\\n"
  2. The board answers "OK", or "AUTH <nonce>" when it has a password. Then:
     "200 <cnonce> <md5(md5(password):nonce:cnonce)>\\n", answered "OK".
  3. The board connects to our TCP port; we send the image in chunks and it
     acknowledges each with the running byte count.
  4. Once its Updater has checked the MD5 it sends "OK" and reboots. Anything
     else is an error, and the old image (or env) stays.

This is a flashing path, so it runs on the robot computer -- the machine that
runs the agent, which for a Wi-Fi robot is the computer it talks to -- exactly
like the USB flasher (docs/firmware.md).
"""
import argparse
import hashlib
import os
import select
import socket
import sys
import time

FLASH, SPIFFS, AUTH = 0, 100, 200
COMMANDS = {"app": FLASH, "env": SPIFFS}
CHUNK = 1460            # one TCP segment; ArduinoOTA acknowledges each write


class OtaError(RuntimeError):
    """The board refused or the transfer failed; nothing new was installed."""


def _md5(data) -> str:
    return hashlib.md5(data if isinstance(data, bytes) else data.encode()).hexdigest()


def local_address_for(host: str) -> str:
    """The address of this machine on the route to `host` -- what the board must
    connect back to. A connected UDP socket sends nothing; it only asks the
    routing table."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def auth_response(password: str, nonce: str, filename: str, size: int,
                  file_md5: str, host: str) -> tuple:
    """(cnonce, response) for the board's AUTH challenge: the MD5 digest scheme
    ArduinoOTA checks, md5(md5(password):nonce:cnonce)."""
    cnonce = _md5(f"{filename}{size}{file_md5}{host}")
    return cnonce, _md5(f"{_md5(password)}:{nonce}:{cnonce}")


def _invite(host: str, port: int, message: str, timeout: float, tries: int = 3) -> str:
    """Send one UDP datagram and return the board's answer. Retried: the radio
    drops the odd datagram, and the board ignores a repeat while it is busy."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        for _ in range(tries):
            sock.sendto(message.encode(), (host, port))
            try:
                return sock.recv(128).decode(errors="replace").strip()
            except socket.timeout:
                continue
        raise OtaError(f"no answer from {host}:{port} -- the board is not on the network, "
                       f"is not running an image with OTA, or a firewall drops UDP {port}")
    finally:
        sock.close()


def upload(host: str, data: bytes, kind: str = "app", port: int = 3232,
           password: str = "", timeout: float = 10.0, progress=None,
           filename: str = "firmware.bin") -> None:
    """Write `data` to the board at `host`. Raises OtaError; returns once the
    board has said OK (it reboots on its own)."""
    if kind not in COMMANDS:
        raise ValueError(f"kind must be one of {sorted(COMMANDS)}")
    size = len(data)
    file_md5 = _md5(data)
    local = local_address_for(host)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((local, 0))
    server.listen(1)
    server.settimeout(timeout)
    try:
        tcp_port = server.getsockname()[1]
        answer = _invite(host, port, f"{COMMANDS[kind]} {tcp_port} {size} {file_md5}\n", timeout)
        if answer.startswith("AUTH"):
            if not password:
                raise OtaError("the board asks for an OTA password and none is set for this robot")
            nonce = answer.split()[1] if len(answer.split()) > 1 else ""
            cnonce, response = auth_response(password, nonce, filename, size, file_md5, host)
            answer = _invite(host, port, f"{AUTH} {cnonce} {response}\n", timeout, tries=1)
            if answer != "OK":
                raise OtaError("the board refused the OTA password" if "Auth" in answer
                               else f"authentication failed: {answer!r}")
        elif answer != "OK":
            raise OtaError(f"the board refused the {kind} write: {answer!r}"
                           + (" -- its partition table or image predates env-over-OTA; "
                              "flash it once over USB" if kind == "env" else ""))
        try:
            conn, _ = server.accept()
        except socket.timeout:
            raise OtaError(f"the board did not connect back to {local}:{tcp_port} -- "
                           f"a firewall on this machine is blocking it")
        with conn:
            conn.settimeout(timeout)       # bounds sendall() when the board stops reading
            sent = 0
            last_ack = time.time()
            while sent < size:
                chunk = data[sent:sent + CHUNK]
                try:
                    conn.sendall(chunk)
                except socket.timeout:
                    raise OtaError(f"the board stopped reading at {sent}/{size} bytes")
                sent += len(chunk)
                # The board answers each Updater write with its byte count, and one
                # write can take several of our chunks -- so the counts are drained
                # as they come rather than awaited one per chunk, and only a board
                # silent for the whole timeout is an error.
                while select.select([conn], [], [], 0)[0]:
                    if not conn.recv(64):
                        raise OtaError(f"the board closed the connection at {sent}/{size} bytes")
                    last_ack = time.time()
                if time.time() - last_ack > timeout:
                    raise OtaError(f"the board stopped acknowledging at {sent}/{size} bytes")
                if progress:
                    progress(sent, size)
            # The Updater checks the MD5, then answers. Writing the last sector
            # and verifying take a few seconds on a 1.9 MB image.
            conn.settimeout(60)
            tail = ""
            deadline = time.time() + 60
            while time.time() < deadline:
                try:
                    got = conn.recv(64)
                except socket.timeout:
                    break
                if not got:
                    break
                tail += got.decode(errors="replace")
                if "OK" in tail:
                    return
                if "rror" in tail or "ERR" in tail:
                    break
            raise OtaError(f"the board did not confirm the {kind} write"
                           + (f": {tail.strip()!r}" if tail.strip() else "")
                           + " -- the previous one stays in place")
    finally:
        server.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("host", help="the board's IP address")
    ap.add_argument("file", help="firmware.bin (--kind app) or an env image (--kind env)")
    ap.add_argument("--kind", choices=sorted(COMMANDS), default="app")
    ap.add_argument("--port", type=int, default=3232)
    ap.add_argument("--password-env", default="LINO_OTA_PASSWORD",
                    help="environment variable holding the OTA password (never a "
                         "command-line argument: every `ps` would show it)")
    a = ap.parse_args()
    data = open(a.file, "rb").read()

    def show(done, total):
        print(f"\r  {done * 100 // total:3d}%  {done}/{total} bytes", end="", flush=True)
    try:
        upload(a.host, data, a.kind, a.port, os.environ.get(a.password_env, ""), progress=show)
    except OtaError as exc:
        print(f"\n❌ {exc}")
        return 1
    print(f"\n✅ {a.kind} written to {a.host}; the board is rebooting into it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
