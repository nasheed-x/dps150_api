#!/usr/bin/env python3
"""
FNIRSI DPS-150 MCP Server

Exposes the DPS-150 power supply as MCP tools for LLM-driven control.

Requires: Python 3.10+, fastmcp (`pip install fastmcp`), pyserial

Run:
    python dps150_mcp.py                      # stdio transport (default)
    python dps150_mcp.py --transport sse      # SSE transport for web clients

Or configure in Claude Code's MCP settings:
    {
        "mcpServers": {
            "dps150": {
                "command": "python3",
                "args": ["dps150_mcp.py"]
            }
        }
    }
"""

import json
from typing import Optional

from fastmcp import FastMCP

from dps150 import DPS150

mcp = FastMCP(
    "FNIRSI DPS-150 Power Supply",
    instructions=(
        "Controls an FNIRSI DPS-150 programmable DC power supply via USB serial. "
        "The DPS-150 outputs 0-24V at 0-5A. Always connect() first, then use "
        "other tools. The device has built-in safety: all voltage/current values "
        "are validated against the device's reported maximums. disconnect() "
        "automatically turns the output off. Protection thresholds (OVP, OCP, "
        "OPP) can be set to prevent damage to connected circuits."
    ),
)

# Global device handle — one connection at a time
_psu: Optional[DPS150] = None


def _require_connection() -> DPS150:
    if _psu is None:
        raise RuntimeError("Not connected. Call connect() first.")
    return _psu


def _fmt(value: float, decimals: int = 3) -> float:
    """Round a float for clean JSON output."""
    return round(value, decimals)


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


@mcp.tool()
def connect(port: str) -> str:
    """Connect to the DPS-150 power supply.

    Opens a USB serial session, reads device info and full state, and caches
    the device's voltage/current maximums for validation.

    Args:
        port: Serial port path, e.g. "/dev/cu.usbmodemXXX" (macOS)
              or "/dev/ttyACM0" (Linux) or "COM3" (Windows).
    """
    global _psu
    if _psu is not None:
        return json.dumps({"error": "Already connected. disconnect() first."})

    psu = DPS150(port)
    psu.connect()
    _psu = psu

    return json.dumps({
        "status": "connected",
        "model": psu.model,
        "firmware": psu.firmware_version,
        "hardware": psu.hardware_version,
        "max_voltage": _fmt(psu.max_voltage, 1),
        "max_current": _fmt(psu.max_current, 1),
    })


@mcp.tool()
def disconnect() -> str:
    """Disconnect from the DPS-150 power supply.

    Turns the output OFF before closing the session. Safe to call even if
    the output is already off.
    """
    global _psu
    if _psu is None:
        return json.dumps({"status": "already disconnected"})

    _psu.disconnect()
    _psu = None
    return json.dumps({"status": "disconnected", "output": "off"})


@mcp.tool()
def read_state() -> str:
    """Read the full state of the DPS-150.

    Returns all device registers as a JSON object including: input voltage,
    voltage/current setpoints, measured output V/I/P, temperature, preset
    values (M1-M6), protection thresholds (OVP/OCP/OPP/OTP/LVP), output
    state (on/off), protection status, and CV/CC mode.
    """
    psu = _require_connection()
    state = psu.read_state()
    if state is None:
        return json.dumps({"error": "Failed to read state from device"})

    # Round floats for readability
    for key in ["input_voltage", "voltage_setpoint", "output_voltage",
                "ovp", "lvp", "max_voltage", "ovp_ceiling", "lvp_ceiling"]:
        if key in state:
            state[key] = _fmt(state[key], 2)
    for key in ["current_setpoint", "output_current", "ocp", "max_current",
                "ocp_ceiling"]:
        if key in state:
            state[key] = _fmt(state[key], 3)
    for key in ["output_power", "opp", "opp_ceiling"]:
        if key in state:
            state[key] = _fmt(state[key], 2)
    for key in ["temperature", "otp", "otp_ceiling"]:
        if key in state:
            state[key] = _fmt(state[key], 1)
    for key in ["ah_counter", "wh_counter"]:
        if key in state:
            state[key] = _fmt(state[key], 3)
    for p in state.get("presets", []):
        p["voltage"] = _fmt(p["voltage"], 2)
        p["current"] = _fmt(p["current"], 3)

    return json.dumps(state)


@mcp.tool()
def set_voltage(volts: float) -> str:
    """Set the voltage setpoint on the DPS-150.

    Writes to register 0xC1 (§5.2). The value is validated against the
    device's maximum voltage capability (typically 24V for DPS-150).
    This only changes the setpoint — it does not enable the output.

    Args:
        volts: Desired voltage in volts (0 to max_voltage).
    """
    psu = _require_connection()
    psu.set_voltage(volts)
    return json.dumps({"status": "ok", "voltage_setpoint": _fmt(volts, 3)})


@mcp.tool()
def set_current(amps: float) -> str:
    """Set the current limit on the DPS-150.

    Writes to register 0xC2 (§5.2). The value is validated against the
    device's maximum current capability (typically 5A for DPS-150).
    This only changes the limit — it does not enable the output.

    Args:
        amps: Desired current limit in amps (0 to max_current).
    """
    psu = _require_connection()
    psu.set_current(amps)
    return json.dumps({"status": "ok", "current_setpoint": _fmt(amps, 3)})


@mcp.tool()
def set_output(volts: float, amps: float) -> str:
    """Set voltage and current, then enable the output.

    Safety: always sets V and A *before* enabling output to prevent
    transient overshoot. Both values are validated against device maximums.

    Args:
        volts: Desired voltage in volts.
        amps: Desired current limit in amps.
    """
    psu = _require_connection()
    psu.set_output(volts, amps)
    return json.dumps({
        "status": "ok",
        "output": "on",
        "voltage_setpoint": _fmt(volts, 3),
        "current_setpoint": _fmt(amps, 3),
    })


@mcp.tool()
def output_on() -> str:
    """Enable the DPS-150 output.

    Writes 1 to register 0xDB (§5.2). The output will use whatever
    voltage and current setpoints are currently configured.
    """
    psu = _require_connection()
    psu.output_on()
    return json.dumps({"status": "ok", "output": "on"})


@mcp.tool()
def output_off() -> str:
    """Disable the DPS-150 output.

    Writes 0 to register 0xDB (§5.2). The voltage and current setpoints
    are preserved but the output is turned off.
    """
    psu = _require_connection()
    psu.output_off()
    return json.dumps({"status": "ok", "output": "off"})


@mcp.tool()
def set_ovp(volts: float) -> str:
    """Set the over-voltage protection threshold.

    Writes to register 0xD1 (§5.2). Validated against the device's OVP
    ceiling (d36, typically ~25V). When output voltage exceeds this
    threshold, the device shuts off and reports OVP status.

    Args:
        volts: OVP threshold in volts.
    """
    psu = _require_connection()
    psu.set_ovp(volts)
    return json.dumps({"status": "ok", "ovp": _fmt(volts, 2)})


@mcp.tool()
def set_ocp(amps: float) -> str:
    """Set the over-current protection threshold.

    Writes to register 0xD2 (§5.2). Validated against the device's OCP
    ceiling (d37). When output current exceeds this threshold, the device
    shuts off and reports OCP status.

    Args:
        amps: OCP threshold in amps.
    """
    psu = _require_connection()
    psu.set_ocp(amps)
    return json.dumps({"status": "ok", "ocp": _fmt(amps, 3)})


@mcp.tool()
def set_opp(watts: float) -> str:
    """Set the over-power protection threshold.

    Writes to register 0xD3 (§5.2). Validated against the device's OPP
    ceiling (d38). When output power exceeds this threshold, the device
    shuts off and reports OPP status.

    Args:
        watts: OPP threshold in watts.
    """
    psu = _require_connection()
    psu.set_opp(watts)
    return json.dumps({"status": "ok", "opp": _fmt(watts, 2)})


@mcp.tool()
def sweep_voltage(
    start_v: float,
    stop_v: float,
    step_v: float,
    hold_s: float,
    current_limit: float,
) -> str:
    """Sweep voltage from start to stop, measuring at each step.

    PC-side automation (§11 "Current Scan"): sets a fixed current limit,
    then steps the voltage from start_v to stop_v. At each step, waits
    hold_s seconds then reads the measured output V, I, P.

    Output is enabled at the start and disabled at the end.

    Args:
        start_v: Starting voltage in volts.
        stop_v: Ending voltage in volts.
        step_v: Voltage step size in volts (must be positive).
        hold_s: Hold time at each step in seconds.
        current_limit: Fixed current limit in amps for the sweep.

    Returns:
        JSON with a list of readings at each step.
    """
    psu = _require_connection()
    results = psu.sweep_voltage(start_v, stop_v, step_v, hold_s, current_limit)
    for r in results:
        r["voltage"] = _fmt(r["voltage"])
        r["current"] = _fmt(r["current"])
        r["power"] = _fmt(r["power"])
    return json.dumps({"status": "ok", "output": "off", "readings": results})


@mcp.tool()
def sweep_current(
    start_a: float,
    stop_a: float,
    step_a: float,
    hold_s: float,
    voltage: float,
) -> str:
    """Sweep current from start to stop, measuring at each step.

    PC-side automation (§11 "Voltage Scan"): sets a fixed voltage,
    then steps the current limit from start_a to stop_a. At each step,
    waits hold_s seconds then reads the measured output V, I, P.

    Output is enabled at the start and disabled at the end.

    Args:
        start_a: Starting current in amps.
        stop_a: Ending current in amps.
        step_a: Current step size in amps (must be positive).
        hold_s: Hold time at each step in seconds.
        voltage: Fixed voltage in volts for the sweep.

    Returns:
        JSON with a list of readings at each step.
    """
    psu = _require_connection()
    results = psu.sweep_current(start_a, stop_a, step_a, hold_s, voltage)
    for r in results:
        r["voltage"] = _fmt(r["voltage"])
        r["current"] = _fmt(r["current"])
        r["power"] = _fmt(r["power"])
    return json.dumps({"status": "ok", "output": "off", "readings": results})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    mcp.run()
