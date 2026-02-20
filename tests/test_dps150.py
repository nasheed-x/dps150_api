"""DPS150 class tests -- uses mock_psu fixture."""

import struct
from unittest.mock import MagicMock, call, patch

import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dps150 import (
    DPS150,
    CMD_WRITE,
    REG_W_VOLTAGE,
    REG_W_CURRENT,
    REG_W_OUTPUT,
    REG_W_OVP,
    REG_W_OCP,
    REG_W_OPP,
    REG_W_OTP,
    REG_W_LVP,
    REG_W_BRIGHTNESS,
    REG_OUTPUT_VIP,
    REG_TEMPERATURE,
    REG_OUTPUT_STATE,
    REG_PROTECTION,
    PROTECTION_NAMES,
    encode_float,
    parse_float,
)


# ---------------------------------------------------------------------------
# Validation tests
# ---------------------------------------------------------------------------

class TestValidation:

    def test_set_voltage_negative(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_voltage(-1.0)

    def test_set_voltage_over_max(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_voltage(25.0)  # max is 24.0

    def test_set_voltage_at_max(self, mock_psu):
        mock_psu.set_voltage(24.0)  # should not raise

    def test_set_current_negative(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_current(-0.1)

    def test_set_current_over_max(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_current(5.1)  # max is 5.0

    def test_set_ovp_over_ceiling(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_ovp(26.0)  # ceiling is 25.0

    def test_set_ovp_negative(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_ovp(-1.0)

    def test_set_ocp_over_ceiling(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_ocp(6.0)  # ceiling is 5.5

    def test_set_opp_over_ceiling(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_opp(131.0)  # ceiling is 130.0

    def test_set_preset_invalid_n_zero(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_preset(0, 5.0, 1.0)

    def test_set_preset_invalid_n_seven(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_preset(7, 5.0, 1.0)

    def test_set_preset_voltage_over_max(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_preset(1, 25.0, 1.0)

    def test_set_preset_current_over_max(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_preset(1, 5.0, 5.1)

    def test_set_brightness_negative(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_brightness(-1)

    def test_set_brightness_over_255(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_brightness(256)

    def test_set_brightness_valid(self, mock_psu):
        mock_psu.set_brightness(128)  # should not raise

    def test_set_volume_out_of_range(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.set_volume(256)


# ---------------------------------------------------------------------------
# State parsing
# ---------------------------------------------------------------------------

class TestStateParsing:

    def test_read_state_values(self, mock_psu):
        """read_state returns correct values from the fake payload."""
        mock_psu._ser.in_waiting = 0
        mock_psu._ser.read = MagicMock(return_value=b"")

        state = mock_psu.read_state()
        assert state is not None

        assert abs(state["input_voltage"] - 12.0) < 0.01
        assert abs(state["voltage_setpoint"] - 5.0) < 0.01
        assert abs(state["current_setpoint"] - 1.0) < 0.01
        assert abs(state["output_voltage"] - 4.99) < 0.01
        assert abs(state["output_current"] - 0.5) < 0.01
        assert abs(state["output_power"] - 2.495) < 0.01
        assert abs(state["temperature"] - 35.0) < 0.1
        assert state["output_on"] is True
        assert state["protection_code"] == 0
        assert state["protection"] == "OK"
        assert state["mode"] == "CV"
        assert abs(state["max_voltage"] - 24.0) < 0.01
        assert abs(state["max_current"] - 5.0) < 0.01

    def test_read_state_presets(self, mock_psu):
        mock_psu._ser.in_waiting = 0
        mock_psu._ser.read = MagicMock(return_value=b"")

        state = mock_psu.read_state()
        assert state is not None
        presets = state["presets"]
        assert len(presets) == 6
        for p in presets:
            assert abs(p["voltage"] - 5.0) < 0.01
            assert abs(p["current"] - 1.0) < 0.01

    def test_read_state_protection_names(self, mock_psu):
        assert PROTECTION_NAMES == ["OK", "OVP", "OCP", "OPP", "OTP", "LVP", "REP"]

    def test_read_state_ceilings(self, mock_psu):
        mock_psu._ser.in_waiting = 0
        mock_psu._ser.read = MagicMock(return_value=b"")

        state = mock_psu.read_state()
        assert state is not None
        assert abs(state["ovp_ceiling"] - 25.0) < 0.01
        assert abs(state["ocp_ceiling"] - 5.5) < 0.01
        assert abs(state["opp_ceiling"] - 130.0) < 0.1
        assert abs(state["otp_ceiling"] - 80.0) < 0.1
        assert abs(state["lvp_ceiling"] - 3.0) < 0.01


# ---------------------------------------------------------------------------
# Output control
# ---------------------------------------------------------------------------

class TestOutputControl:

    def test_set_output_calls_in_order(self, mock_psu):
        """set_output calls set_voltage, set_current, output_on in that order."""
        mock_psu.set_voltage = MagicMock()
        mock_psu.set_current = MagicMock()
        mock_psu.output_on = MagicMock()

        mock_psu.set_output(5.0, 1.0)

        mock_psu.set_voltage.assert_called_once_with(5.0)
        mock_psu.set_current.assert_called_once_with(1.0)
        mock_psu.output_on.assert_called_once()

    def test_output_on_sends_correct_register(self, mock_psu):
        mock_psu.output_on()
        mock_psu._send.assert_called_once()
        args = mock_psu._send.call_args
        assert args[0][0] == CMD_WRITE
        assert args[0][1] == REG_W_OUTPUT
        assert args[0][2] == bytes([0x01])

    def test_output_off_sends_correct_register(self, mock_psu):
        mock_psu.output_off()
        mock_psu._send.assert_called_once()
        args = mock_psu._send.call_args
        assert args[0][0] == CMD_WRITE
        assert args[0][1] == REG_W_OUTPUT
        assert args[0][2] == bytes([0x00])


# ---------------------------------------------------------------------------
# Sweep validation
# ---------------------------------------------------------------------------

class TestSweepValidation:

    def test_sweep_voltage_negative_step(self, mock_psu):
        with pytest.raises(ValueError, match="step_v must be positive"):
            mock_psu.sweep_voltage(0, 5, -0.1, 0.1, 1.0)

    def test_sweep_voltage_zero_step(self, mock_psu):
        with pytest.raises(ValueError, match="step_v must be positive"):
            mock_psu.sweep_voltage(0, 5, 0, 0.1, 1.0)

    def test_sweep_voltage_over_max_start(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.sweep_voltage(25, 5, 1, 0.1, 1.0)

    def test_sweep_voltage_over_max_stop(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.sweep_voltage(0, 25, 1, 0.1, 1.0)

    def test_sweep_current_negative_step(self, mock_psu):
        with pytest.raises(ValueError, match="step_a must be positive"):
            mock_psu.sweep_current(0, 3, -0.1, 0.1, 5.0)

    def test_sweep_current_over_max_range(self, mock_psu):
        with pytest.raises(ValueError):
            mock_psu.sweep_current(0, 6, 0.5, 0.1, 5.0)


# ---------------------------------------------------------------------------
# Telemetry parsing
# ---------------------------------------------------------------------------

class TestTelemetryParsing:

    def _make_telem_packet(self, register, payload):
        """Build a telemetry response packet: F0 A1 <reg> <len> <payload> <checksum>"""
        length = len(payload)
        checksum = (register + length + sum(payload)) & 0xFF
        return bytes([0xF0, 0xA1, register, length]) + payload + bytes([checksum])

    def test_parse_vip(self):
        """Register 0xC3 -- output V/I/P (3 floats)."""
        payload = encode_float(4.99) + encode_float(0.5) + encode_float(2.495)
        pkt = self._make_telem_packet(0xC3, payload)
        result = DPS150._parse_telemetry_packet(pkt)
        assert result is not None
        assert abs(result["output_voltage"] - 4.99) < 0.01
        assert abs(result["output_current"] - 0.5) < 0.01
        assert abs(result["output_power"] - 2.495) < 0.01

    def test_parse_temperature(self):
        """Register 0xC4 -- temperature."""
        payload = encode_float(35.0)
        pkt = self._make_telem_packet(0xC4, payload)
        result = DPS150._parse_telemetry_packet(pkt)
        assert result is not None
        assert abs(result["temperature"] - 35.0) < 0.1

    def test_parse_output_state(self):
        """Register 0xDB -- output on/off."""
        pkt = self._make_telem_packet(0xDB, bytes([0x01]))
        result = DPS150._parse_telemetry_packet(pkt)
        assert result is not None
        assert result["output_on"] is True

        pkt_off = self._make_telem_packet(0xDB, bytes([0x00]))
        result_off = DPS150._parse_telemetry_packet(pkt_off)
        assert result_off["output_on"] is False

    def test_parse_protection_ok(self):
        """Register 0xDC -- protection code 0 (OK)."""
        pkt = self._make_telem_packet(0xDC, bytes([0x00]))
        result = DPS150._parse_telemetry_packet(pkt)
        assert result is not None
        assert result["protection_code"] == 0
        assert result["protection"] == "OK"

    def test_parse_protection_ovp(self):
        """Register 0xDC -- protection code 1 (OVP)."""
        pkt = self._make_telem_packet(0xDC, bytes([0x01]))
        result = DPS150._parse_telemetry_packet(pkt)
        assert result["protection_code"] == 1
        assert result["protection"] == "OVP"

    def test_parse_protection_all_codes(self):
        """Register 0xDC -- all protection codes 0-6."""
        for code, name in enumerate(PROTECTION_NAMES):
            pkt = self._make_telem_packet(0xDC, bytes([code]))
            result = DPS150._parse_telemetry_packet(pkt)
            assert result["protection_code"] == code
            assert result["protection"] == name

    def test_parse_too_short(self):
        """Packet shorter than 5 bytes returns None."""
        assert DPS150._parse_telemetry_packet(bytes([0xF0, 0xA1, 0xC3])) is None

    def test_parse_unknown_register(self):
        """Unknown register returns None."""
        pkt = self._make_telem_packet(0xAA, bytes([0x01]))
        assert DPS150._parse_telemetry_packet(pkt) is None
