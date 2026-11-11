"""The 8x8 ToF frame core (scripts/tof_matrix.py): parsing, decoding, orientation, projection."""
import math

import pytest

import tof_matrix as tm


def flat(d):
    return [d] * 64


def test_zone_angles_span_the_field_symmetrically():
    a = [tm.zone_angle_deg(i) for i in range(8)]
    assert a[0] == pytest.approx(-26.25) and a[-1] == pytest.approx(26.25)
    assert a[3] == pytest.approx(-3.75) and a[4] == pytest.approx(3.75)


def test_a_flat_wall_square_on_is_a_plane_at_its_distance():
    # perpendicular distance: every zone of a wall 1 m away reads 1000, every point has x = 1 m
    pts = tm.to_points(flat(1000))
    assert len(pts) == 64
    assert all(p[0] == pytest.approx(1.0) for p in pts)
    corner = max(abs(p[1]) for p in pts)
    assert corner == pytest.approx(math.tan(math.radians(26.25)))


def test_invalid_and_zero_zones_are_dropped():
    f = flat(1000)
    f[0] = 4000
    f[1] = 0
    f[2] = 5           # 5 mm: under the minimum range
    assert len(tm.to_points(f)) == 61


def test_beyond_max_range_is_dropped():
    assert tm.to_points(flat(3600)) == []
    assert len(tm.to_points(flat(3600), max_range_m=4.0)) == 64


def test_row0_col0_is_top_left_of_the_view():
    f = flat(4000)
    f[0] = 1000                         # row 0, col 0
    (x, y, z), = tm.to_points(f)
    assert x == pytest.approx(1.0) and y > 0 and z > 0     # left (+y) and up (+z)
    f = flat(4000)
    f[63] = 1000                        # row 7, col 7: bottom right
    (x, y, z), = tm.to_points(f)
    assert y < 0 and z < 0


@pytest.mark.parametrize("rotation,mirror,expect_index", [
    (0, False, 0), (90, False, 7), (180, False, 63), (270, False, 56), (0, True, 7),
])
def test_orient_moves_the_corner(rotation, mirror, expect_index):
    f = list(range(64))
    assert tm.orient(f, rotation, mirror)[expect_index] == 0


def test_four_quarter_turns_are_identity():
    f = list(range(64))
    g = f
    for _ in range(4):
        g = tm.orient(g, 90)
    assert g == f


def test_orient_refuses_odd_angles():
    with pytest.raises(ValueError):
        tm.orient(flat(1), 45)


def test_raw_frame_round_trip():
    f = [i * 50 for i in range(64)]
    raw = tm.encode_raw(f)
    assert len(raw) == tm.FRAME_BYTES == 128
    assert tm.decode_raw(raw) == f
    with pytest.raises(ValueError):
        tm.decode_raw(raw[:-2])


def _usb_frame(values_by_row):
    return "".join(f"y{r}:" + ",".join(str(v) for v in values_by_row[r]) + ",\n" for r in range(8)).encode()


def test_parser_assembles_whole_frames_and_skips_the_partial_start():
    rows = [[r * 100 + c for c in range(8)] for r in range(8)]
    stream = b"3,4000,4000,\ny6:" + b",".join(b"1" for _ in range(8)) + b",\n" + _usb_frame(rows) * 2
    p = tm.FrameParser()
    frames = []
    for i in range(0, len(stream), 7):            # arbitrary chunking, like a serial read
        frames += p.feed(stream[i:i + 7])
    assert len(frames) == 2
    assert frames[0] == [d for r in rows for d in r]


def test_parser_drops_a_frame_with_a_missing_row():
    rows = [[1000] * 8 for _ in range(8)]
    text = _usb_frame(rows).decode().splitlines(keepends=True)
    del text[4]                                    # y4 lost
    p = tm.FrameParser()
    assert p.feed("".join(text).encode()) == []
    assert p.feed(_usb_frame(rows)) == [[1000] * 64]


def test_parser_drops_a_short_row():
    p = tm.FrameParser()
    bad = _usb_frame([[1000] * 8 for _ in range(8)]).replace(b"y3:1000,", b"y3:", 1)
    assert p.feed(bad) == []
    assert p.bad_lines == 1


def test_the_real_boards_lines_parse():
    # a frame as the board streams it on its USB port
    sample = (b"y0:4000,4000,2551,4000,4000,4000,4000,4000,\n"
              b"y1:4000,4000,4000,4000,4000,4000,1640,1196,\n"
              b"y2:4000,4000,4000,4000,4000,4000,4000,4000,\n"
              b"y3:2482,4000,4000,4000,4000,4000,4000,1351,\n"
              b"y4:4000,4000,4000,2523,4000,4000,4000,1254,\n"
              b"y5:1455,4000,4000,4000,4000,4000,1518,1142,\n"
              b"y6:1434,1437,4000,4000,4000,1516,1346,1036,\n"
              b"y7:1383,1411,1421,1421,1394,1480,1132,875,\n")
    frames = tm.FrameParser().feed(sample)
    assert len(frames) == 1 and frames[0][63] == 875
    assert len(tm.to_points(frames[0])) == sum(1 for d in frames[0] if d != 4000) == 23


def test_pointcloud2_packing():
    fields, step, data = tm.pack_pointcloud2_xyz([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    assert step == 12 and len(data) == 24
    assert [f[0] for f in fields] == ["x", "y", "z"]
