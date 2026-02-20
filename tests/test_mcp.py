"""MCP server tool tests -- calls @mcp.tool() functions directly."""

import json
from unittest.mock import MagicMock, patch

import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import dps150_mcp
from dps150 import DPS150


@pytest.fixture(autouse=True)
def reset_psu():
    """Reset global _psu before each test."""
    dps150_mcp._psu = None
    yield
    dps150_mcp._psu = None


def _make_mock_psu():
    """Create a mock DPS150 with all properties."""
    psu = MagicMock(spec=DPS150)
    psu.model = "DPS-150"
    psu.firmware_version = "1.0.0"
    psu.hardware_version = "2.0"
    psu.max_voltage = 24.0
    psu.max_current = 5.0
    return psu


# ---------------------------------------------------------------------------
# connect / disconnect
# ---------------------------------------------------------------------------

class TestConnect:

    @patch("dps150_mcp.DPS150")
    def test_connect(self, MockDPS150):
        instance = _make_mock_psu()
        MockDPS150.return_value = instance

        result = json.loads(dps150_mcp.connect("/dev/fake"))
        assert result["status"] == "connected"
        assert result["model"] == "DPS-150"
        assert result["firmware"] == "1.0.0"
        instance.connect.assert_called_once()

    @patch("dps150_mcp.DPS150")
    def test_double_connect(self, MockDPS150):
        instance = _make_mock_psu()
        MockDPS150.return_value = instance

        dps150_mcp.connect("/dev/fake")
        result = json.loads(dps150_mcp.connect("/dev/fake"))
        assert "error" in result


class TestDisconnect:

    def test_disconnect_not_connected(self):
        result = json.loads(dps150_mcp.disconnect())
        assert result["status"] == "already disconnected"

    @patch("dps150_mcp.DPS150")
    def test_disconnect_connected(self, MockDPS150):
        instance = _make_mock_psu()
        MockDPS150.return_value = instance
        dps150_mcp.connect("/dev/fake")

        result = json.loads(dps150_mcp.disconnect())
        assert result["status"] == "disconnected"
        instance.disconnect.assert_called_once()


# ---------------------------------------------------------------------------
# read_state
# ---------------------------------------------------------------------------

class TestReadState:

    def test_read_state(self):
        psu = _make_mock_psu()
        psu.read_state.return_value = {
            "input_voltage": 12.001,
            "voltage_setpoint": 5.001,
            "current_setpoint": 1.0005,
            "output_voltage": 4.991,
            "output_current": 0.5003,
            "output_power": 2.4955,
            "temperature": 35.05,
            "presets": [
                {"voltage": 5.001, "current": 1.0005} for _ in range(6)
            ],
            "ovp": 25.001,
            "ocp": 5.5003,
            "opp": 130.005,
            "otp": 80.05,
            "lvp": 3.001,
            "brightness": 128,
            "volume": 64,
            "metering": "stopped",
            "ah_counter": 0.0,
            "wh_counter": 0.0,
            "output_on": True,
            "protection_code": 0,
            "protection": "OK",
            "mode": "CV",
            "max_voltage": 24.001,
            "max_current": 5.0005,
            "ovp_ceiling": 25.001,
            "ocp_ceiling": 5.5003,
            "opp_ceiling": 130.005,
            "otp_ceiling": 80.05,
            "lvp_ceiling": 3.001,
        }
        dps150_mcp._psu = psu

        result = json.loads(dps150_mcp.read_state())
        assert "input_voltage" in result
        assert result["output_on"] is True
        assert result["protection"] == "OK"
        assert result["mode"] == "CV"


# ---------------------------------------------------------------------------
# Setters
# ---------------------------------------------------------------------------

class TestSetters:

    def _setup_psu(self):
        psu = _make_mock_psu()
        dps150_mcp._psu = psu
        return psu

    def test_set_voltage(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_voltage(5.0))
        assert result["status"] == "ok"
        psu.set_voltage.assert_called_once_with(5.0)

    def test_set_current(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_current(1.0))
        assert result["status"] == "ok"
        psu.set_current.assert_called_once_with(1.0)

    def test_set_output(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_output(5.0, 1.0))
        assert result["status"] == "ok"
        assert result["output"] == "on"
        psu.set_output.assert_called_once_with(5.0, 1.0)

    def test_output_on(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.output_on())
        assert result["status"] == "ok"
        assert result["output"] == "on"
        psu.output_on.assert_called_once()

    def test_output_off(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.output_off())
        assert result["status"] == "ok"
        assert result["output"] == "off"
        psu.output_off.assert_called_once()

    def test_set_ovp(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_ovp(20.0))
        assert result["status"] == "ok"
        psu.set_ovp.assert_called_once_with(20.0)

    def test_set_ocp(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_ocp(4.0))
        assert result["status"] == "ok"
        psu.set_ocp.assert_called_once_with(4.0)

    def test_set_opp(self):
        psu = self._setup_psu()
        result = json.loads(dps150_mcp.set_opp(100.0))
        assert result["status"] == "ok"
        psu.set_opp.assert_called_once_with(100.0)


# ---------------------------------------------------------------------------
# Without connection -- raises RuntimeError
# ---------------------------------------------------------------------------

class TestWithoutConnection:

    def test_read_state_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.read_state()

    def test_set_voltage_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.set_voltage(5.0)

    def test_set_current_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.set_current(1.0)

    def test_set_output_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.set_output(5.0, 1.0)

    def test_output_on_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.output_on()

    def test_output_off_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.output_off()

    def test_set_ovp_no_connection(self):
        with pytest.raises(RuntimeError, match="Not connected"):
            dps150_mcp.set_ovp(20.0)
