"""A saved map's .pgm as a PNG a browser can show.

map_saver_cli writes the occupancy grid as a PGM (P5), and no browser decodes
PGM: the Saved Maps gallery pointed <img> at the .pgm and every thumbnail was a
broken image. This turns it into a small grayscale PNG with the standard library
alone (zlib + struct), so the robot computer needs no imaging package.
"""
import struct
import zlib


class PgmError(ValueError):
    pass


def parse_pgm(data: bytes):
    """(width, height, maxval, pixels) of a P5 (binary) or P2 (ASCII) PGM.

    `pixels` is one byte per pixel, row-major, top row first; a 16-bit image is
    scaled down to 8 bits.
    """
    if data[:2] not in (b"P5", b"P2"):
        raise PgmError("not a PGM (no P5/P2 magic)")
    magic = data[:2]
    tokens = []
    i = 2
    n = len(data)
    while len(tokens) < 3:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if i < n and data[i:i + 1] == b"#":           # a comment runs to the end of the line
            while i < n and data[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        start = i
        while i < n and not data[i:i + 1].isspace() and data[i:i + 1] != b"#":
            i += 1
        if start == i:
            raise PgmError("truncated PGM header")
        tokens.append(data[start:i])
    try:
        width, height, maxval = (int(t) for t in tokens)
    except ValueError:
        raise PgmError("bad PGM header")
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise PgmError(f"bad PGM size {width}x{height} maxval {maxval}")
    count = width * height
    if magic == b"P5":
        i += 1                                         # exactly one whitespace byte after maxval
        if maxval < 256:
            pixels = data[i:i + count]
            if len(pixels) < count:
                raise PgmError("truncated PGM data")
            if maxval != 255:
                pixels = bytes(min(255, p * 255 // maxval) for p in pixels)
        else:
            raw = data[i:i + 2 * count]
            if len(raw) < 2 * count:
                raise PgmError("truncated PGM data")
            vals = struct.unpack(f">{count}H", raw)
            pixels = bytes(v * 255 // maxval for v in vals)
    else:
        vals = data[i:].split()[:count]
        if len(vals) < count:
            raise PgmError("truncated PGM data")
        pixels = bytes(min(255, int(v) * 255 // maxval) for v in vals)
    return width, height, maxval, pixels


def _chunk(kind: bytes, body: bytes) -> bytes:
    return (struct.pack(">I", len(body)) + kind + body
            + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))


def gray_png(width: int, height: int, pixels: bytes) -> bytes:
    """An 8-bit grayscale PNG of `pixels` (row-major, one byte each)."""
    rows = b"".join(b"\x00" + pixels[y * width:(y + 1) * width] for y in range(height))
    return (b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + _chunk(b"IDAT", zlib.compress(rows, 6))
            + _chunk(b"IEND", b""))


def pgm_to_png(data: bytes, max_side: int = 512) -> bytes:
    """The PGM as a PNG, shrunk (every n-th pixel) so neither side exceeds max_side.

    A SLAM map can be thousands of cells a side; a thumbnail does not need them,
    and nearest-neighbour keeps walls black instead of averaging them away.
    """
    width, height, _, pixels = parse_pgm(data)
    step = 1
    if max_side and max(width, height) > max_side:
        step = -(-max(width, height) // max_side)     # ceil
    if step > 1:
        rows = [pixels[y * width:(y + 1) * width:step] for y in range(0, height, step)]
        width, height = len(rows[0]), len(rows)
        pixels = b"".join(rows)
    return gray_png(width, height, pixels)
