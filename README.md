# DPS-150 API

Python API and MCP server for the [FNIRSI DPS-150](https://www.fnirsi.cn/) programmable DC power supply (0-24V, 0-5A).

Designed for automated testing workflows where an LLM (via MCP) or a Python script needs to control a bench power supply with built-in safety checks.

## Files

| File | Description |
|------|-------------|
| `dps150.py` | Python library — single-file `DPS150` class wrapping all serial communication |
| `dps150_mcp.py` | MCP server — exposes the library as tools for Claude Code, Codex, Cursor, Amp, etc. |
| `PROTOCOL.md` | Full protocol specification reverse-engineered from the official FNIRSI Windows app |

## Requirements

- Python 3.10+
- [pyserial](https://pypi.org/project/pyserial/) (`pip install pyserial`)
- [fastmcp](https://pypi.org/project/fastmcp/) (`pip install fastmcp`) — only for the MCP server

## Quick Start

### Python Library

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

Output is automatically turned off when the context manager exits (even on exceptions).

### Command Line

```bash
# Quick self-test: connect, read state, disconnect
python dps150.py /dev/cu.usbmodemXXX
```

### MCP Server

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

## Protocol

See [PROTOCOL.md](PROTOCOL.md) for the full serial protocol specification, including packet structure, register map, command bytes, and data formats. All register addresses and offsets in the code reference sections from this document.
