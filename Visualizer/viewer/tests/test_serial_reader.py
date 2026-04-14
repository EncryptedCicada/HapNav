"""Tests for serial_reader module."""

import numpy as np
import pytest

from viewer.serial_reader import SerialReader
from viewer import config


class TestSerialReaderValidation:
    """Tests for data validation methods."""

    def test_validate_distances_valid(self):
        """Should accept valid distance values."""
        reader = SerialReader("/dev/null")

        assert reader._validate_distances([100, 200, 300])
        assert reader._validate_distances([0, 4000, 2000])
        assert reader._validate_distances([100.5, 200.5, 300.5])

    def test_validate_distances_nan(self):
        """Should reject NaN values."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_distances([100, float('nan'), 300])

    def test_validate_distances_inf(self):
        """Should reject Inf values."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_distances([100, float('inf'), 300])
        assert not reader._validate_distances([100, float('-inf'), 300])

    def test_validate_distances_non_numeric(self):
        """Should reject non-numeric values."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_distances([100, "bad", 300])
        assert not reader._validate_distances([100, None, 300])

    def test_validate_quaternion_valid(self):
        """Should accept valid quaternion values."""
        reader = SerialReader("/dev/null")

        assert reader._validate_quaternion([1, 0, 0, 0])
        assert reader._validate_quaternion([0.707, 0, 0, 0.707])
        assert reader._validate_quaternion([0.5, 0.5, 0.5, 0.5])

    def test_validate_quaternion_wrong_length(self):
        """Should reject quaternions with wrong length."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_quaternion([1, 0, 0])
        assert not reader._validate_quaternion([1, 0, 0, 0, 0])

    def test_validate_quaternion_nan(self):
        """Should reject NaN in quaternion."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_quaternion([1, float('nan'), 0, 0])

    def test_validate_quaternion_inf(self):
        """Should reject Inf in quaternion."""
        reader = SerialReader("/dev/null")

        assert not reader._validate_quaternion([float('inf'), 0, 0, 0])


class TestSerialReaderInitialization:
    """Tests for SerialReader initialization."""

    def test_initializes_with_defaults(self):
        """Should initialize with default values."""
        reader = SerialReader("/dev/test")

        assert reader.port == "/dev/test"
        assert reader.baud == 115200
        assert not reader.running
        assert reader.distances.shape == (config.NUM_ZONES,)
        assert reader.status.shape == (config.NUM_ZONES,)
        assert reader.quaternion.shape == (4,)

    def test_initializes_with_custom_baud(self):
        """Should accept custom baud rate."""
        reader = SerialReader("/dev/test", baud=9600)

        assert reader.baud == 9600

    def test_initial_quaternion_is_identity(self):
        """Initial quaternion should be identity."""
        reader = SerialReader("/dev/test")

        np.testing.assert_array_equal(
            reader.quaternion,
            [1.0, 0.0, 0.0, 0.0]
        )

    def test_data_fps_starts_at_zero(self):
        """Data FPS should start at zero."""
        reader = SerialReader("/dev/test")

        assert reader.data_fps == 0.0


class TestSerialReaderGetData:
    """Tests for get_data method."""

    def test_returns_copies(self):
        """get_data should return copies, not references."""
        reader = SerialReader("/dev/test")
        reader.distances[0] = 100

        distances, status, quat = reader.get_data()

        # Modify returned array
        distances[0] = 999

        # Original should be unchanged
        assert reader.distances[0] == 100

    def test_returns_correct_types(self):
        """Returned arrays should have correct dtypes."""
        reader = SerialReader("/dev/test")

        distances, status, quat = reader.get_data()

        assert distances.dtype == np.float32
        assert status.dtype == np.uint8
        assert quat.dtype == np.float32
