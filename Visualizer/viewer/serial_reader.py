"""Serial reader for the HapNav pin's PIN-prefixed JSON trace stream.

The pin emits one line per sample, e.g.

  PIN {"ts":4763,"a":[...],"g":[...],"q":[w,x,y,z],
       "head":mm_or_-1,"down":mm_or_-1,
       "obs":{"urg":[u0,u1,u2,u3],"near":mm_or_-1,"flags":"0xNN"},
       "d":[..64..],"st":[..64..]}

`a`/`g`/`obs`/`ts` are conveniences the firmware prints; we only need `q`,
`d`/`st` for the front grid, and `head`/`down` for the single-zone ToFs.

The 8×8 grid is only emitted on every ~20th sample (the firmware's
`dump_grid` divisor); the reader caches the last-good grid in between so
downstream code always has *some* current data to render.
"""

import json
import logging
import math
import threading
import time

import numpy as np
import serial

from . import config

logger = logging.getLogger("hapnav_pin_viewer.serial")


_PIN_PREFIX = "PIN "


class SerialReader:
    """Background thread for reading PIN-prefixed JSON data from the pin."""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False

        self._lock = threading.Lock()

        # Front grid (cached — emitted every ~20 samples).
        self.front_distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.front_status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self._front_seen = False

        # Single-zone heads — refreshed every sample.
        self.head_distance_mm: float = -1.0
        self.down_distance_mm: float = -1.0

        # Body→world quaternion from the BNO055 (already remapped by firmware).
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._imu_connected = False

        # Obstacle overlay — drives the on-screen status panel.
        self.urgency = np.zeros(4, dtype=np.uint8)
        self.nearest_mm: int = -1
        self.flags: int = 0

        # FPS tracking.
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._fps = 0.0

        self._thread: threading.Thread | None = None

    # ── public read-side API ────────────────────────────────────────────

    @property
    def data_fps(self) -> float:
        with self._lock:
            return self._fps

    @property
    def imu_connected(self) -> bool:
        with self._lock:
            return self._imu_connected

    @property
    def front_grid_received(self) -> bool:
        """True once at least one grid frame has been parsed."""
        with self._lock:
            return self._front_seen

    def get_frame(self) -> dict:
        """Snapshot all latest sensor state. Returns a dict so callers can
        ignore fields they don't care about; arrays are copies."""
        with self._lock:
            return {
                "front_distances": self.front_distances.copy(),
                "front_status": self.front_status.copy(),
                "front_seen": self._front_seen,
                "head_distance_mm": self.head_distance_mm,
                "down_distance_mm": self.down_distance_mm,
                "quaternion": self.quaternion.copy(),
                "imu_connected": self._imu_connected,
                "urgency": self.urgency.copy(),
                "nearest_mm": self.nearest_mm,
                "flags": self.flags,
            }

    # ── connection / thread lifecycle ───────────────────────────────────

    def connect(self):
        logger.info("Connecting to %s at %d baud...", self.port, self.baud)
        self.serial = serial.Serial(self.port, self.baud, timeout=1)
        # The pin's CONFIG_BOOT_DELAY is 2 s; matching it here lets the
        # boot banner drain before we start parsing.
        time.sleep(2)
        self.serial.reset_input_buffer()
        logger.info("Serial connected")

    def start(self):
        if self._thread is not None:
            return
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    # ── validation helpers ──────────────────────────────────────────────

    @staticmethod
    def _is_finite_seq(seq) -> bool:
        for v in seq:
            if not isinstance(v, (int, float)):
                return False
            if math.isnan(v) or math.isinf(v):
                return False
        return True

    @staticmethod
    def _parse_flags(value) -> int:
        if isinstance(value, int):
            return value
        if isinstance(value, str):
            try:
                return int(value, 0)
            except ValueError:
                return 0
        return 0

    # ── reconnection ────────────────────────────────────────────────────

    def _reconnect(self) -> bool:
        try:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.serial.reset_input_buffer()
            logger.info("Serial reconnected")
            return True
        except (serial.SerialException, OSError) as e:
            logger.debug("Reconnection failed: %s", e)
            return False

    # ── main read loop ──────────────────────────────────────────────────

    def _read_loop(self):
        logger.info("Serial reader thread started")
        while self.running:
            try:
                if not (self.serial and self.serial.is_open):
                    time.sleep(0.1)
                    continue

                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line.startswith(_PIN_PREFIX):
                    continue

                payload = line[len(_PIN_PREFIX):].strip()
                try:
                    data = json.loads(payload)
                except json.JSONDecodeError:
                    # Common in the wild — interleaved logs, partial lines.
                    continue

                self._consume(data)

            except (serial.SerialException, OSError) as e:
                if not self.running:
                    break
                logger.warning("Serial connection lost: %s", e)
                with self._lock:
                    self._fps = 0.0
                while self.running:
                    if self._reconnect():
                        break
                    time.sleep(1)

    def _consume(self, data: dict):
        """Validate + stash one parsed PIN frame."""
        quat = data.get("q")
        head = data.get("head")
        down = data.get("down")
        d = data.get("d")
        st = data.get("st")
        obs = data.get("obs", {})

        new_grid_valid = False
        if isinstance(d, list) and isinstance(st, list):
            if len(d) == config.NUM_ZONES and len(st) == config.NUM_ZONES:
                if self._is_finite_seq(d):
                    new_grid_valid = True

        with self._lock:
            if isinstance(quat, list) and len(quat) == 4 and self._is_finite_seq(quat):
                self.quaternion = np.asarray(quat, dtype=np.float32)
                self._imu_connected = True

            if isinstance(head, (int, float)) and not math.isnan(head):
                self.head_distance_mm = float(head)
            if isinstance(down, (int, float)) and not math.isnan(down):
                self.down_distance_mm = float(down)

            if new_grid_valid:
                self.front_distances = np.asarray(d, dtype=np.float32)
                self.front_status = np.asarray(st, dtype=np.uint8)
                self._front_seen = True

            if isinstance(obs, dict):
                urg = obs.get("urg")
                if isinstance(urg, list) and len(urg) == 4 and self._is_finite_seq(urg):
                    self.urgency = np.asarray(urg, dtype=np.uint8)
                near = obs.get("near")
                if isinstance(near, (int, float)) and not math.isnan(near):
                    self.nearest_mm = int(near)
                self.flags = self._parse_flags(obs.get("flags", 0))

            self._frame_count += 1
            now = time.time()
            elapsed = now - self._last_fps_time
            if elapsed >= 1.0:
                self._fps = self._frame_count / elapsed
                self._frame_count = 0
                self._last_fps_time = now
