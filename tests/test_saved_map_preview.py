"""Saved Maps thumbnails are PNGs, because no browser shows a PGM.

map_saver_cli writes the grid as a .pgm and the gallery pointed <img> straight
at it: every thumbnail was a broken image (web UI walkthrough, 2026-10-16). The
backend now serves /api/maps/<name>.pgm/preview, the PGM re-encoded as a small
grayscale PNG with the standard library alone.
"""
import os
import struct
import zlib

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _pgm(width, height, pixels, comment=b"# CREATOR: map_saver.cpp 0.050 m/pix\n"):
    return b"P5\n" + comment + f"{width} {height}\n255\n".encode() + bytes(pixels)


def _decode_png(png):
    """(width, height, rows) of an 8-bit grayscale, unfiltered PNG."""
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    i, idat, ihdr = 8, b"", None
    while i < len(png):
        (length,) = struct.unpack(">I", png[i:i + 4])
        kind, body = png[i + 4:i + 8], png[i + 8:i + 8 + length]
        (crc,) = struct.unpack(">I", png[i + 8 + length:i + 12 + length])
        assert crc == zlib.crc32(kind + body) & 0xFFFFFFFF
        if kind == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat += body
        i += 12 + length
    w, h, depth, color = ihdr[:4]
    assert (depth, color) == (8, 0)
    raw = zlib.decompress(idat)
    rows = [raw[y * (w + 1) + 1:(y + 1) * (w + 1)] for y in range(h)]
    assert all(raw[y * (w + 1)] == 0 for y in range(h))
    return w, h, rows


def test_a_map_saver_pgm_becomes_a_png_with_the_same_pixels():
    import map_preview
    px = [0, 205, 254, 0, 254, 205]                  # occupied, unknown, free
    w, h, rows = _decode_png(map_preview.pgm_to_png(_pgm(3, 2, px)))
    assert (w, h) == (3, 2)
    assert rows == [bytes(px[:3]), bytes(px[3:])]


def test_a_large_map_is_shrunk_to_the_thumbnail():
    import map_preview
    w, h, rows = _decode_png(map_preview.pgm_to_png(_pgm(1000, 400, [254] * 400000), max_side=100))
    assert max(w, h) <= 100 and w == 100 and h == 40


def test_ascii_and_16_bit_pgms_read_too():
    import map_preview
    assert map_preview.parse_pgm(b"P2\n2 1\n255\n0 255\n")[3] == bytes([0, 255])
    data = b"P5 2 1 65535\n" + struct.pack(">2H", 0, 65535)
    assert map_preview.parse_pgm(data)[3] == bytes([0, 255])


def test_a_bad_file_is_an_error_not_a_crash():
    import map_preview
    for bad in (b"", b"GIF89a", b"P5\n3 2\n255\n\x00"):
        with pytest.raises(map_preview.PgmError):
            map_preview.parse_pgm(bad)


def test_the_route_serves_a_png_and_refuses_paths(tmp_path, monkeypatch):
    pytest.importorskip("fastapi")
    pytest.importorskip("httpx", reason="FastAPI's TestClient needs httpx")
    from fastapi.testclient import TestClient
    import routes_hardware

    (tmp_path / "maps").mkdir()
    (tmp_path / "maps" / "room.pgm").write_bytes(_pgm(2, 2, [0, 254, 205, 0]))
    monkeypatch.setattr(routes_hardware, "REPO_ROOT", str(tmp_path))
    client = TestClient(routes_hardware.app)
    r = client.get("/api/maps/room.pgm/preview")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/png"
    assert _decode_png(r.content)[:2] == (2, 2)
    assert client.get("/api/maps/none.pgm/preview").status_code == 404
    assert client.get("/api/maps/room.yaml/preview").status_code == 404
    assert client.get("/api/maps/..%2Fsecrets.yaml").status_code in (400, 404)


def test_the_route_function_without_a_client(tmp_path, monkeypatch):
    """The same route called directly, for a backend without httpx."""
    fastapi = pytest.importorskip("fastapi")
    import routes_hardware

    (tmp_path / "maps").mkdir()
    (tmp_path / "maps" / "room.pgm").write_bytes(_pgm(2, 2, [0, 254, 205, 0]))
    (tmp_path / "maps" / "junk.pgm").write_bytes(b"not a map")
    monkeypatch.setattr(routes_hardware, "REPO_ROOT", str(tmp_path))
    r = routes_hardware.get_map_preview("room.pgm", size=512)
    assert r.media_type == "image/png"
    assert _decode_png(r.body)[:2] == (2, 2)
    for name, code in (("none.pgm", 404), ("junk.pgm", 422), ("..", 400)):
        with pytest.raises(fastapi.HTTPException) as e:
            routes_hardware.get_map_preview(name, size=512)
        assert e.value.status_code == code, name


def test_the_gallery_asks_for_the_png():
    js = open(os.path.join(REPO_ROOT, "web", "frontend", "app-workflow.js")).read()
    body = js[js.index("async function loadMaps"):]
    body = body[:body.index("\n  }\n")]
    assert "/preview" in body.split("<img src=", 1)[1].split(">", 1)[0]
