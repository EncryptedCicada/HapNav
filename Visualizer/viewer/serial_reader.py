"""Serial communication with VL53L5CX sensor via ESP32."""

import json
import logging
import math
import threading
import time

import numpy as np
import serial

from . import config

logger = logging.getLogger("vl53l5cx_viewer.serial")


class SerialReader:
    """Background thread for reading sensor data over serial."""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.serial: serial.Serial | None = None
        self.running = False

        # Data storage
        self.distances = np.zeros(config.NUM_ZONES, dtype=np.float32)
        self.status = np.zeros(config.NUM_ZONES, dtype=np.uint8)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # wxyz identity
        self._imu_connected = False  # True if IMU data has been received
        self._data_lock = threading.Lock()

        # FPS tracking
        self._frame_count = 0
        self._last_fps_time = time.time()
        self._data_fps = 0.0

        self._thread: threading.Thread | None = None
        self._version_checked = False

    @property
    def data_fps(self) -> float:
        """Current data frame rate from sensor."""
        with self._data_lock:
            return self._data_fps

    @property
    def imu_connected(self) -> bool:
        """True if IMU data has been received (quat field present in serial data)."""
        with self._data_lock:
            return self._imu_connected

    def connect(self):
        """Open serial connection."""
        logger.info("Connecting to %s at %d baud...", self.port, self.baud)
        self.serial = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)  # Wait for ESP32 to initialize
        self.serial.reset_input_buffer()
        logger.info("Serial connected")

    def start(self):
        """Start the reader thread."""
        if self._thread is not None:
            return

        self.running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the reader thread and close serial."""
        self.running = False
        if self.serial:
            self.serial.close()
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def get_data(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get a copy of the latest distance, status, and quaternion data.

        Returns:
            Tuple of (distances, status, quaternion) arrays
        """
        with self._data_lock:
            return self.distances.copy(), self.status.copy(), self.quaternion.copy()

    def _validate_distances(self, distances: list) -> bool:
        """Validate distance values are within expected range."""
        for d in distances:
            if not isinstance(d, (int, float)):
                return False
            if math.isnan(d) or math.isinf(d):
                return False
        return True

    def _validate_quaternion(self, quat: list) -> bool:
        """Validate quaternion values."""
        if len(quat) != 4:
            return False
        for q in quat:
            if not isinstance(q, (int, float)):
                return False
            if math.isnan(q) or math.isinf(q):
                return False
        return True

    def _reconnect(self) -> bool:
        """Attempt to reconnect to serial port.

        Returns:
            True if reconnection successful, False otherwise
        """
        try:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for ESP32/sensor to initialize
            self.serial.reset_input_buffer()
            logger.info("Serial reconnected")
            return True
        except (serial.SerialException, OSError) as e:
            logger.debug("Reconnection failed: %s", e)
            return False

    def _read_loop(self):
        """Background thread to read serial data."""
        logger.info("Serial reader thread started")
        while self.running:
            try:
                if self.serial and self.serial.is_open:
                    line = self.serial.readline()
                    if line:
                        line_str = line.decode("utf-8", errors="ignore").strip()
                        if line_str.startswith("{"):
                            try:
                                data = json.loads(line_str)
                                if "distances" in data and "status" in data:
                                    distances = data["distances"]
                                    status = data["status"]
                                    # Validate array lengths to handle corrupted serial data
                                    if len(distances) != config.NUM_ZONES or len(status) != config.NUM_ZONES:
                                        logger.warning(
                                            "Invalid array lengths: distances=%d, status=%d (expected %d)",
                                            len(distances), len(status), config.NUM_ZONES
                                        )
                                        continue
                                    # Validate distance values
                                    if not self._validate_distances(distances):
                                        logger.warning("Invalid distance values detected (NaN/Inf)")
                                        continue
                                    # Validate quaternion if present
                                    if "quat" in data and not self._validate_quaternion(data["quat"]):
                                        logger.warning("Invalid quaternion values detected (NaN/Inf)")
                                        data.pop("quat")  # Still process distances, skip bad quaternion
                                    # Check version (warn once)
                                    if not self._version_checked:
                                        self._version_checked = True
                                        firmware_version = data.get("v")
                                        if firmware_version is None:
                                            logger.warning(
                                                "No version in data. "
                                                "Firmware may be outdated - consider reflashing."
                                            )
                                        elif firmware_version != config.VERSION:
                                            logger.warning(
                                                "Version mismatch: firmware=%s, viewer=%s. "
                                                "Consider reflashing the ESP32.",
                                                firmware_version, config.VERSION
                                            )
                                    with self._data_lock:
                                        self.distances = np.array(distances, dtype=np.float32)
                                        self.status = np.array(status, dtype=np.uint8)
                                        if "quat" in data:
                                            self.quaternion = np.array(
                                                data["quat"], dtype=np.float32
                                            )
                                            self._imu_connected = True
                                        else:
                                            logger.debug("No quaternion data in packet (IMU may not be connected or firmware outdated)")
                                    # Track data FPS
                                    self._frame_count += 1
                                    now = time.time()
                                    elapsed = now - self._last_fps_time
                                    if elapsed >= 1.0:
                                        with self._data_lock:
                                            self._data_fps = self._frame_count / elapsed
                                        self._frame_count = 0
                                        self._last_fps_time = now
                            except json.JSONDecodeError as e:
                                logger.debug("JSON decode error: %s", e)
            except (serial.SerialException, OSError) as e:
                if not self.running:
                    break
                logger.warning("Serial connection lost: %s", e)
                with self._data_lock:
                    self._data_fps = 0.0  # Reset FPS indicator
                while self.running:
                    if self._reconnect():
                        break
                    time.sleep(1)  # Wait before retry
