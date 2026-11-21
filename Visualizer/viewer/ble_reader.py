"""BLE data source: live frames from the HapNav chest pin in laptop-link mode.

Drop-in alternative to SerialReader (same connect/start/stop/get_data
interface and attributes), selected with `python -m viewer --ble`.

Pairs with pin firmware built with CONFIG_HAPNAV_LINK_LAPTOP=y
(ZephyrProject/Firmware/pin/prj_laptop.conf). Wire format: packed
`struct hapnav_frame` from lib/ble/include/hapnav/ble_proto.h, sent as
GATT notifications with a 1-byte [seq<<4 | chunk_idx] header. The same
decoding lives in Tools/ble_receiver.py (CLI logger); keep them in sync
if the struct changes.
"""

from __future__ import annotations

import asyncio
import datetime as dt
import json
import logging
import math
import struct
import threading
import time
from pathlib import Path

import numpy as np

from . import config

logger = logging.getLogger("vl53l5cx_viewer.ble")

SVC_UUID = "5a9c0010-7e6f-4abc-9c12-aabbccdd1100"
FRAME_UUID = "5a9c0011-7e6f-4abc-9c12-aabbccdd1100"
DEFAULT_NAME = "HapNav-Pin"

# u32 ts | f32 quat[4] | i16 dist[64] | u8 status[64] | u8 urg[4] | i16 near | u8 flags
FRAME_FMT = "<I4f64h64B4BhB"
FRAME_SIZE = struct.calcsize(FRAME_FMT)
assert FRAME_SIZE == 219, FRAME_SIZE

FLAG_NAMES = {
    0x01: "STATIONARY",
    0x02: "SENSOR_BLOCKED",
    0x04: "MOSTLY_INVALID",
    0x08: "YAW_SLEWING",
    0x10: "DROPOFF",
    0x20: "HEAD_OBSTACLE",
}

# <repo>/Logs — shared with Tools/serial_logger.py and Tools/ble_receiver.py
DEFAULT_LOG_DIR = Path(__file__).resolve().parents[2] / "Logs"


def decode_frame(raw: bytes) -> dict:
    v = struct.unpack(FRAME_FMT, raw)
    return {
        "ts": v[0],
        "quat": list(v[1:5]),
        "distances": list(v[5:69]),
        "status": list(v[69:133]),
        "urg": list(v[133:137]),
        "near": v[137],
        "flags": v[138],
    }


VALID_STATUS = 5  # VL53L5CX target_status meaning "valid measurement"


def format_tof_grid(f: dict, rate_hz: float = 0.0) -> str:
    """Render a decoded frame as a header line plus the 8x8 distance grid (mm).

    Rows/columns follow the firmware's row-major zone order (zone 0 = row 0,
    col 0). Zones whose status isn't VALID_STATUS print as '----'.
    """
    flags = ",".join(n for bit, n in FLAG_NAMES.items() if f["flags"] & bit) or "-"
    lines = [
        f"ToF ts={f['ts']} ms  {rate_hz:4.1f} Hz  near={f['near']} mm  "
        f"urg={f['urg']}  flags=0x{f['flags']:02x}({flags})"
    ]
    for row in range(8):
        cells = []
        for col in range(8):
            i = row * 8 + col
            cells.append(f"{f['distances'][i]:5d}" if f["status"][i] == VALID_STATUS else " ----")
        lines.append(f"  r{row} " + " ".join(cells))
    return "\n".join(lines)


class Reassembler:
    """Rebuilds frames from [hdr: seq<<4 | idx][payload] notifications."""

    def __init__(self) -> None:
        self.buf: bytearray | None = None
        self.seq = -1
        self.expect = 0
        self.dropped = 0

    def feed(self, data: bytes) -> bytes | None:
        if not data:
            return None
        seq, idx, payload = data[0] >> 4, data[0] & 0x0F, data[1:]
        if idx == 0:
            if self.buf is not None:
                self.dropped += 1
            self.buf, self.seq, self.expect = bytearray(payload), seq, 1
        elif self.buf is not None and seq == self.seq and idx == self.expect:
            self.buf += payload
            self.expect += 1
        else:
            if self.buf is not None:
                self.dropped += 1
            self.buf = None
            return None
        if len(self.buf) >= FRAME_SIZE:
            frame, self.buf = bytes(self.buf[:FRAME_SIZE]), None
            return frame
        return None


class BleReader:
    """Background thread receiving pin frames over BLE (bleak)."""

    def __init__(self, name: str = DEFAULT_NAME, scan_timeout: float = 10.0,
                 log_dir: Path | str | None = None, print_every: int = 0):
        """log_dir: if set, every received frame is appended as one JSON line
        to <log_dir>/ble_viewer_<timestamp>.jsonl (same record format as
        Tools/ble_receiver.py).
        print_every: if > 0, print the 8x8 ToF grid to stdout every N frames."""
        self.name = name
        self.print_every = max(0, int(print_every))
        self._print_count = 0
        self.scan_timeout = scan_timeout
        self.running = False
        self.log_dir = Path(log_dir) if log_dir else None
        self.log_path: Path | None = None
        self._log_file = None

        # Same public state as SerialReader
        self.distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # wxyz
        self._imu_connected = False
        self.head_mm: int = -1   # not carried in the BLE frame
        self.down_mm: int = -1   # not carried in the BLE frame
        self.obs_flags: int = 0
        self.urgency = [0, 0, 0, 0]
        self.nearest_mm: int = -1
        self._data_lock = threading.Lock()

        self._frame_count = 0
        self._last_fps_time = time.time()
        self._data_fps = 0.0

        self._reasm = Reassembler()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_evt: asyncio.Event | None = None

    # ── SerialReader-compatible interface ──────────────────────────────────

    @property
    def data_fps(self) -> float:
        with self._data_lock:
            return self._data_fps

    @property
    def imu_connected(self) -> bool:
        with self._data_lock:
            return self._imu_connected

    def connect(self):
        """No-op: scanning/connecting happens in the background thread."""
        logger.info("BLE mode: will scan for %s (%s)", self.name, SVC_UUID)

    def start(self):
        if self._thread is not None:
            return
        self._open_log()
        self.running = True
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._loop and self._stop_evt:
            self._loop.call_soon_threadsafe(self._stop_evt.set)
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        self._close_log()

    # ── frame log ──────────────────────────────────────────────────────────

    def _open_log(self):
        if self.log_dir is None or self._log_file is not None:
            return
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.log_dir / f"ble_viewer_{dt.datetime.now():%Y%m%d_%H%M%S}.jsonl"
        self._log_file = self.log_path.open("w", buffering=1)  # line-buffered
        logger.info("Logging frames to %s", self.log_path)

    def _close_log(self):
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
            logger.info("Frame log closed: %s", self.log_path)

    def _write_log(self, f: dict):
        record = {
            "host_t": round(time.time(), 4),
            **f,
            "quat": [round(q, 4) for q in f["quat"]],
            "flag_names": [n for bit, n in FLAG_NAMES.items() if f["flags"] & bit],
        }
        try:
            self._log_file.write(json.dumps(record) + "\n")
        except (OSError, ValueError) as e:  # disk full / file closed mid-shutdown
            logger.warning("Frame log write failed (%s); logging disabled", e)
            self._log_file = None

    def get_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._data_lock:
            return self.distances.copy(), self.status.copy(), self.quaternion.copy()

    # ── frame handling ─────────────────────────────────────────────────────

    def _on_notify(self, _char, data: bytearray) -> None:
        raw = self._reasm.feed(bytes(data))
        if raw is None:
            return
        f = decode_frame(raw)
        if self._log_file is not None:
            self._write_log(f)
        if self.print_every:
            self._print_count += 1
            if self._print_count >= self.print_every:
                self._print_count = 0
                print(format_tof_grid(f, self.data_fps), flush=True)
        quat_ok = all(math.isfinite(q) for q in f["quat"]) and any(f["quat"])

        with self._data_lock:
            self.distances = np.array(f["distances"], dtype=np.float32)
            self.status = np.array(f["status"], dtype=np.uint8)
            if quat_ok:
                self.quaternion = np.array(f["quat"], dtype=np.float32)
                self._imu_connected = True
            self.obs_flags = f["flags"]
            self.urgency = f["urg"]
            self.nearest_mm = f["near"]

        self._frame_count += 1
        now = time.time()
        if now - self._last_fps_time >= 1.0:
            with self._data_lock:
                self._data_fps = self._frame_count / (now - self._last_fps_time)
            self._frame_count, self._last_fps_time = 0, now

    # ── BLE session management ─────────────────────────────────────────────

    def _thread_main(self):
        try:
            asyncio.run(self._main())
        except Exception:
            logger.exception("BLE thread crashed")

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        self._stop_evt = asyncio.Event()
        while self.running:
            try:
                await self._session()
            except Exception as e:  # BLE errors are routine; retry
                logger.warning("BLE error: %r", e)
            with self._data_lock:
                self._data_fps = 0.0
            if self.running:
                try:
                    await asyncio.wait_for(self._stop_evt.wait(), timeout=2.0)
                except asyncio.TimeoutError:
                    pass

    async def _session(self):
        from bleak import BleakClient, BleakScanner  # optional dependency

        logger.info("Scanning for %s ...", self.name)
        dev = await BleakScanner.find_device_by_filter(
            lambda d, adv: SVC_UUID in [u.lower() for u in adv.service_uuids]
            or (d.name or adv.local_name) == self.name,
            timeout=self.scan_timeout,
        )
        if dev is None:
            logger.info("Pin not found; retrying")
            return

        lost = asyncio.Event()
        async with BleakClient(dev, disconnected_callback=lambda _c: lost.set()) as client:
            logger.info("Connected to %s [%s], MTU %d", dev.name, dev.address, client.mtu_size)
            self._reasm = Reassembler()
            await client.start_notify(FRAME_UUID, self._on_notify)
            waiters = [asyncio.create_task(lost.wait()),
                       asyncio.create_task(self._stop_evt.wait())]
            _, pending = await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
        if self.running:
            logger.warning("BLE link lost; reconnecting")
