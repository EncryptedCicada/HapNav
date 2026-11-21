"""Tests for ble_reader: frame decoding, chunk reassembly, state updates."""

import struct

import numpy as np
import pytest

from viewer.ble_reader import FRAME_FMT, FRAME_SIZE, BleReader, Reassembler, decode_frame


def make_frame(ts=1234, quat=(1.0, 0.0, 0.0, 0.0), flags=0x11, near=-1):
    dist = list(range(100, 164))
    status = [5] * 64
    return struct.pack(FRAME_FMT, ts, *quat, *dist, *status, 10, 20, 30, 255, near, flags)


def chunks(frame: bytes, seq: int, mtu: int) -> list[bytes]:
    """Mirror of pin_ble_send_frame() in pin/src/ble_laptop.c."""
    max_chunk = min(mtu - 3 - 1, 244)
    out, off, idx = [], 0, 0
    while off < len(frame):
        n = min(max_chunk, len(frame) - off)
        out.append(bytes([((seq & 0xF) << 4) | (idx & 0xF)]) + frame[off:off + n])
        off += n
        idx += 1
    return out


def test_frame_size_matches_firmware_struct():
    assert FRAME_SIZE == 219


def test_decode_frame_fields():
    f = decode_frame(make_frame(ts=42, near=80, flags=0x20))
    assert f["ts"] == 42
    assert f["quat"] == [1.0, 0.0, 0.0, 0.0]
    assert f["distances"][0] == 100 and f["distances"][63] == 163
    assert f["status"] == [5] * 64
    assert f["urg"] == [10, 20, 30, 255]
    assert f["near"] == 80
    assert f["flags"] == 0x20


@pytest.mark.parametrize("mtu", [23, 185, 247])
def test_reassembly_round_trip(mtu):
    frame = make_frame()
    r = Reassembler()
    got = [out for seq in range(20) for c in chunks(frame, seq, mtu) if (out := r.feed(c))]
    assert got == [frame] * 20
    assert r.dropped == 0


def test_lost_chunk_drops_only_that_frame():
    frame = make_frame()
    r = Reassembler()
    got = 0
    for seq in range(5):
        cs = chunks(frame, seq, 23)
        if seq == 2:
            del cs[3]  # lose a middle chunk
        for c in cs:
            if r.feed(c) is not None:
                got += 1
    assert got == 4
    assert r.dropped == 1


def test_on_notify_updates_reader_state():
    reader = BleReader()
    assert not reader.front_grid_received
    for c in chunks(make_frame(quat=(0.0, 1.0, 0.0, 0.0), flags=0x08, near=80), 0, 185):
        reader._on_notify(None, bytearray(c))
    frame = reader.get_frame()
    assert frame["front_distances"][5] == 105
    assert frame["front_status"][0] == 5
    assert frame["front_seen"] and reader.front_grid_received
    np.testing.assert_allclose(frame["quaternion"], [0.0, 1.0, 0.0, 0.0])
    assert frame["imu_connected"] and reader.imu_connected
    assert frame["flags"] == 0x08
    assert list(frame["urgency"]) == [10, 20, 30, 255]
    assert frame["nearest_mm"] == 80
    assert frame["head_distance_mm"] == -1.0 and frame["down_distance_mm"] == -1.0


def test_zero_quaternion_is_ignored():
    reader = BleReader()
    for c in chunks(make_frame(quat=(0.0, 0.0, 0.0, 0.0)), 0, 247):
        reader._on_notify(None, bytearray(c))
    frame = reader.get_frame()
    np.testing.assert_allclose(frame["quaternion"], [1.0, 0.0, 0.0, 0.0])  # identity kept
    assert not frame["imu_connected"]


def test_get_frame_matches_serial_reader_interface():
    """HapNavViewer reads get_frame(); BleReader must expose the same keys
    and types as SerialReader so either can be the viewer's data source."""
    from viewer.serial_reader import SerialReader

    serial_frame = SerialReader("/dev/null").get_frame()
    ble_frame = BleReader().get_frame()
    assert set(ble_frame) == set(serial_frame)
    for key, value in serial_frame.items():
        assert type(ble_frame[key]) is type(value), key
        if isinstance(value, np.ndarray):
            assert ble_frame[key].shape == value.shape, key
            assert ble_frame[key].dtype == value.dtype, key


def test_log_writes_one_json_line_per_frame(tmp_path):
    import json

    reader = BleReader(log_dir=tmp_path)
    reader._open_log()
    for seq in range(3):
        for c in chunks(make_frame(ts=100 + seq, flags=0x11, near=80), seq, 185):
            reader._on_notify(None, bytearray(c))
    reader._close_log()

    files = list(tmp_path.glob("ble_viewer_*.jsonl"))
    assert len(files) == 1
    rows = [json.loads(line) for line in files[0].read_text().splitlines()]
    assert [r["ts"] for r in rows] == [100, 101, 102]
    assert len(rows[0]["distances"]) == 64 and len(rows[0]["status"]) == 64
    assert rows[0]["near"] == 80
    assert rows[0]["flag_names"] == ["STATIONARY", "DROPOFF"]
    assert "host_t" in rows[0]


def test_no_log_file_without_log_dir(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    reader = BleReader()
    reader._open_log()
    for c in chunks(make_frame(), 0, 247):
        reader._on_notify(None, bytearray(c))
    assert reader.log_path is None
    assert list(tmp_path.iterdir()) == []


def test_format_tof_grid_marks_invalid_zones():
    from viewer.ble_reader import format_tof_grid

    f = decode_frame(make_frame(ts=7, near=80, flags=0x01))
    f["status"][9] = 255  # row 1, col 1 invalid
    text = format_tof_grid(f, rate_hz=33.0)
    lines = text.splitlines()
    assert len(lines) == 9  # header + 8 rows
    assert "ts=7" in lines[0] and "near=80" in lines[0] and "STATIONARY" in lines[0]
    assert lines[1].split()[1:3] == ["100", "101"]
    assert lines[2].split()[1:3] == ["108", "----"]


def test_print_every_n_frames(capsys):
    reader = BleReader(print_every=2)
    for seq in range(4):
        for c in chunks(make_frame(ts=seq), seq, 247):
            reader._on_notify(None, bytearray(c))
    out = capsys.readouterr().out
    assert out.count("ToF ts=") == 2  # frames 2 and 4


def test_no_print_by_default(capsys):
    reader = BleReader()
    for c in chunks(make_frame(), 0, 247):
        reader._on_notify(None, bytearray(c))
    assert capsys.readouterr().out == ""
