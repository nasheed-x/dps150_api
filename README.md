# DPS-150 API

Python API and MCP server for the [FNIRSI DPS-150](https://www.fnirsi.cn/) programmable DC power supply (0-24V, 0-5A).

Designed for automated testing workflows where an LLM (via MCP) or a Python script needs to control a bench power supply with built-in safety checks.

## Files

| File | Description |
|------|-------------|
| `dps150.py` | Python library + CLI — single-file `DPS150` class wrapping all serial communication |
| `dps150_mcp.py` | MCP server — exposes the library as tools for Claude Code, Codex, Cursor, Amp, etc. |
| `PROTOCOL.md` | Full protocol specification reverse-engineered from the official FNIRSI Windows app |

## Requirements

- Python 3.10+
- [pyserial](https://pypi.org/project/pyserial/) (`pip install pyserial`)
- [fastmcp](https://pypi.org/project/fastmcp/) (`pip install fastmcp`) — only for the MCP server

## Command Line

Every function in the library is available as a CLI subcommand. Commands that set state (voltage, current, output on, protection thresholds, etc.) leave the device running — only read commands and sweeps turn the output off on exit.

```bash
# Device info
python dps150.py info
python dps150.py -p /dev/ttyACM0 info       # specify port

# Full state as JSON
python dps150.py state

# Read measurements
python dps150.py voltage
python dps150.py current
python dps150.py power
python dps150.py input-voltage
python dps150.py temperature

# Set voltage / current (does NOT enable output)
python dps150.py set-voltage 5.0
python dps150.py set-current 1.0

# Set voltage + current and turn output ON in one command
python dps150.py set-output 5.0 1.0

# Toggle output
python dps150.py on
python dps150.py off

# Protection thresholds
python dps150.py set-ovp 25.0
python dps150.py set-ocp 5.5
python dps150.py set-opp 130.0
python dps150.py set-otp 80.0
python dps150.py set-lvp 3.0

# Presets
python dps150.py presets
python dps150.py set-preset 1 3.3 0.5

# Display / sound
python dps150.py set-brightness 128
python dps150.py set-volume 5

# Metering (Ah/Wh counters)
python dps150.py start-metering
python dps150.py stop-metering

# Voltage sweep: 0→12V, 0.5V steps, 0.5s hold, 1A limit
python dps150.py sweep-voltage 0 12 0.5 0.5 1.0

# Current sweep: 0→3A, 0.1A steps, 0.5s hold, 5V fixed
python dps150.py sweep-current 0 3 0.1 0.5 5.0

# Restart device
python dps150.py restart
```

The default port is `/dev/cu.usbmodem065AD9D205B31`. Override with `-p PORT`.

## Python Library

```python
from dps150 import DPS150

with DPS150("/dev/cu.usbmodemXXX") as psu:
    print(psu.model, psu.firmware_version)
    print(psu.read_state())

    # Set 5V at 1A and enable output
    psu.set_output(5.0, 1.0)

    # Read measurements
    print(f"{psu.read_voltage():.2f}V  {psu.read_current():.3f}A")

    # Voltage sweep: 0-12V in 0.5V steps, 1A limit, 0.5s per step
    results = psu.sweep_voltage(0, 12, 0.5, 0.5, 1.0)
    for r in results:
        print(f"  {r['voltage']:.2f}V  {r['current']:.3f}A  {r['power']:.2f}W")
```

Output is automatically turned off when the context manager exits (even on exceptions). Use `psu.close()` instead of `psu.disconnect()` if you want to detach without turning the output off.

## MCP Server

Add to your MCP client config (Claude Code, Cursor, Amp, Codex, etc.):

```json
{
  "mcpServers": {
    "dps150": {
      "command": "python3",
      "args": ["/path/to/dps150_mcp.py"]
    }
  }
}
```

The server exposes these tools:

| Tool | Description |
|------|-------------|
| `connect(port)` | Open session, read device info |
| `disconnect()` | Output off + close session |
| `read_state()` | Full state dump as JSON |
| `set_voltage(volts)` | Set voltage setpoint |
| `set_current(amps)` | Set current limit |
| `set_output(volts, amps)` | Set V/A then enable output |
| `output_on()` / `output_off()` | Toggle output |
| `set_ovp(volts)` / `set_ocp(amps)` / `set_opp(watts)` | Protection thresholds |
| `sweep_voltage(start_v, stop_v, step_v, hold_s, current_limit)` | Voltage sweep with readings |
| `sweep_current(start_a, stop_a, step_a, hold_s, voltage)` | Current sweep with readings |

## Safety

- All setters validate against the device's reported maximums (read on connect)
- Protection thresholds validate against the device's ceiling values
- `set_output()` always sets voltage and current *before* enabling
- `disconnect()` always turns the output off first
- Context manager `__exit__` calls `disconnect()` even on exceptions
- Sweeps validate the full range before starting and turn output off when done
- CLI commands that set state use `close()` (output stays on); read-only commands and sweeps use `disconnect()` (output off)

## Protocol

See [PROTOCOL.md](PROTOCOL.md) for the full serial protocol specification, including packet structure, register map, command bytes, and data formats. All register addresses and offsets in the code reference sections from this document.
