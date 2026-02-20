#!/usr/bin/env python3
"""
FNIRSI DPS-150 Power Supply — Python API

All register addresses, offsets, command bytes, and data formats are sourced
directly from PROTOCOL.md (the canonical spec for this project).

Requires: pyserial (`pip install pyserial`)
"""

import struct
import threading
import time
from typing import Callable, Optional

import serial

# ---------------------------------------------------------------------------
# Constants — §2 Packet Structure
# ---------------------------------------------------------------------------
HEADER_TX = 0xF1  # host → device
HEADER_RX = 0xF0  # device → host

# §3 Command Categories
CMD_READ = 0xA1
CMD_BAUD = 0xB0
CMD_WRITE = 0xB1
CMD_RESTART = 0xC0
CMD_SESSION = 0xC1

# §5.1 Readable Registers
REG_INPUT_VOLTAGE = 0xC0
REG_VOLTAGE_SET = 0xC1
REG_CURRENT_SET = 0xC2
REG_OUTPUT_VIP = 0xC3  # 3×float32: V, I, P
REG_TEMPERATURE = 0xC4
REG_BRIGHTNESS = 0xD6
REG_VOLUME = 0xD7
REG_AH_COUNTER = 0xD9
REG_WH_COUNTER = 0xDA
REG_OUTPUT_STATE = 0xDB
REG_PROTECTION = 0xDC
REG_CVCC_MODE = 0xDD
REG_MODEL = 0xDE
REG_HW_VERSION = 0xDF
REG_FW_VERSION = 0xE0
REG_DEVICE_ADDR = 0xE1
REG_MAX_VOLTAGE = 0xE2
REG_MAX_CURRENT = 0xE3
REG_ALL = 0xFF

# §5.2 Writable Registers
REG_W_VOLTAGE = 0xC1
REG_W_CURRENT = 0xC2
REG_W_OVP = 0xD1
REG_W_OCP = 0xD2
REG_W_OPP = 0xD3
REG_W_OTP = 0xD4
REG_W_LVP = 0xD5
REG_W_BRIGHTNESS = 0xD6
REG_W_VOLUME = 0xD7
REG_W_METERING = 0xD8
REG_W_OUTPUT = 0xDB

# §5.3 Preset Register Formula: voltage = 0xC3 + 2*n, current = 0xC3 + 2*n + 1
PRESET_REGS = [(0xC3 + 2 * n, 0xC3 + 2 * n + 1) for n in range(1, 7)]

# §6 Protection Status Codes
PROTECTION_NAMES = ["OK", "OVP", "OCP", "OPP", "OTP", "LVP", "REP"]

# Default capabilities before device reports actual values (§7)
DEFAULT_MAX_VOLTAGE = 24.0
DEFAULT_MAX_CURRENT = 5.0

# §4.5 Inter-Command Delay
CMD_DELAY = 0.05  # 50 ms


# ---------------------------------------------------------------------------
# Packet helpers — §2 Packet Structure
# ---------------------------------------------------------------------------
def build_packet(header: int, command: int, register: int, data: bytes = b"") -> bytes:
    """Build a wire packet per §2: header | command | register | length | data | checksum."""
    length = len(data)
    checksum = (register + length + sum(data)) & 0xFF
    return bytes([header, command, register, length]) + data + bytes([checksum])


def parse_responses(buf: bytes) -> list[bytes]:
    """Split a receive buffer into individual packets per §10."""
    packets = []
    i = 0
    while i < len(buf):
        if i + 4 > len(buf):
            break
        pkt_len = 5 + buf[i + 3]
        if i + pkt_len > len(buf):
            break
        packets.append(buf[i : i + pkt_len])
        i += pkt_len
    return packets


def parse_float(data: bytes, offset: int = 0) -> float:
    """Extract IEEE 754 LE float32 per §8."""
    return struct.unpack_from("<f", data, offset)[0]


def encode_float(value: float) -> bytes:
    """Encode a float32 as IEEE 754 LE per §8."""
    return struct.pack("<f", value)


# ---------------------------------------------------------------------------
# DPS150 class
# ---------------------------------------------------------------------------
class DPS150:
    """Python API for the FNIRSI DPS-150 programmable power supply.

    Usage::

        with DPS150("/dev/cu.usbmodemXXX") as psu:
            print(psu.read_state())
            psu.set_output(5.0, 1.0)
    """

    def __init__(self, port: str, baud: int = 115200):
        self._port = port
        self._baud = baud
        self._ser: Optional[serial.Serial] = None

        # Device info (populated on connect)
        self._model: Optional[str] = None
        self._fw_version: Optional[str] = None
        self._hw_version: Optional[str] = None
        self._max_voltage: float = DEFAULT_MAX_VOLTAGE
        self._max_current: float = DEFAULT_MAX_CURRENT

        # Protection ceilings (d36-d40, populated from full state dump)
        self._ovp_ceiling: float = 0.0
        self._ocp_ceiling: float = 0.0
        self._opp_ceiling: float = 0.0
        self._otp_ceiling: float = 0.0
        self._lvp_ceiling: float = 0.0

        # Telemetry thread
        self._telemetry_thread: Optional[threading.Thread] = None
        self._telemetry_stop = threading.Event()
        self._telemetry_callback: Optional[Callable] = None
        self._telemetry_lock = threading.Lock()

    # -- Properties ----------------------------------------------------------

    @property
    def model(self) -> Optional[str]:
        return self._model

    @property
    def firmware_version(self) -> Optional[str]:
        return self._fw_version

    @property
    def hardware_version(self) -> Optional[str]:
        return self._hw_version

    @property
    def max_voltage(self) -> float:
        return self._max_voltage

    @property
    def max_current(self) -> float:
        return self._max_current

    # -- Context manager -----------------------------------------------------

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # -- Low-level I/O -------------------------------------------------------

    def _send(self, command: int, register: int, data: bytes = b""):
        """Send a packet and wait the inter-command delay."""
        pkt = build_packet(HEADER_TX, command, register, data)
        self._ser.write(pkt)
        time.sleep(CMD_DELAY)

    def _drain(self) -> bytes:
        """Read all available bytes from the serial buffer."""
        time.sleep(0.05)
        data = self._ser.read(self._ser.in_waiting or 0)
        return data

    def _send_and_recv(self, command: int, register: int, data: bytes = b"",
                       wait: float = 0.2) -> bytes:
        """Send a packet, wait, then read the response buffer."""
        pkt = build_packet(HEADER_TX, command, register, data)
        self._ser.write(pkt)
        time.sleep(wait)
        resp = self._ser.read(self._ser.in_waiting or 256)
        return resp

    def _find_response(self, buf: bytes, register: int) -> Optional[bytes]:
        """Find the first response packet matching a register in a buffer."""
        for pkt in parse_responses(buf):
            if len(pkt) >= 5 and pkt[2] == register:
                return pkt
        return None

    def _read_string_register(self, register: int) -> Optional[str]:
        """Read a string register (model, firmware, hardware version)."""
        resp = self._send_and_recv(CMD_READ, register, b"\x00", wait=0.5)
        pkt = self._find_response(resp, register)
        if pkt and len(pkt) > 5:
            return pkt[4:-1].decode("ascii", errors="replace")
        return None

    def _write_float(self, register: int, value: float):
        """Write a float32 to a writable register."""
        self._send(CMD_WRITE, register, encode_float(value))

    def _write_byte(self, register: int, value: int):
        """Write a single byte to a writable register."""
        self._send(CMD_WRITE, register, bytes([value & 0xFF]))

    # -- Connection lifecycle (§4) -------------------------------------------

    def connect(self):
        """Open the serial port and initialize the device session.

        Performs the full §4.2 initialization sequence:
        1. Open serial port with RTS asserted
        2. Session open handshake
        3. Set baud rate
        4. Read model, firmware, hardware version
        5. Read full state dump (caches max V/A and protection ceilings)
        """
        self._ser = serial.Serial(
            self._port, self._baud,
            bytesize=8, parity="N", stopbits=1,
            timeout=1, rtscts=False,
        )
        self._ser.rts = True
        time.sleep(0.1)

        # Flush stale data
        self._ser.read(self._ser.in_waiting or 0)

        # §4.1 Session open
        self._send_and_recv(CMD_SESSION, 0x00, b"\x01", wait=0.5)

        # Drain auto-pushed telemetry
        time.sleep(0.3)
        self._ser.read(self._ser.in_waiting or 0)

        # §4.2 Set baud rate (index 5 = 115200)
        self._send(CMD_BAUD, 0x00, b"\x05")

        # Read device info
        self._model = self._read_string_register(REG_MODEL)
        self._fw_version = self._read_string_register(REG_FW_VERSION)
        self._hw_version = self._read_string_register(REG_HW_VERSION)

        # Read full state to cache capabilities and ceilings
        state = self.read_state()
        if state:
            self._max_voltage = state.get("max_voltage", DEFAULT_MAX_VOLTAGE)
            self._max_current = state.get("max_current", DEFAULT_MAX_CURRENT)
            self._ovp_ceiling = state.get("ovp_ceiling", 0.0)
            self._ocp_ceiling = state.get("ocp_ceiling", 0.0)
            self._opp_ceiling = state.get("opp_ceiling", 0.0)
            self._otp_ceiling = state.get("otp_ceiling", 0.0)
            self._lvp_ceiling = state.get("lvp_ceiling", 0.0)

    def disconnect(self):
        """Turn output off and close the session (safe shutdown)."""
        if self._ser and self._ser.is_open:
            self.stop_telemetry()
            try:
                self.output_off()
            except Exception:
                pass
            try:
                self._send_and_recv(CMD_SESSION, 0x00, b"\x00", wait=0.3)
            except Exception:
                pass
            self._ser.close()
        self._ser = None

    def close(self):
        """Close the session without turning output off.

        Use this when you want to detach from the device but leave it
        running in its current state (e.g. CLI one-shot commands).
        """
        if self._ser and self._ser.is_open:
            self.stop_telemetry()
            try:
                self._send_and_recv(CMD_SESSION, 0x00, b"\x00", wait=0.3)
            except Exception:
                pass
            self._ser.close()
        self._ser = None

    # -- State reading (§7) --------------------------------------------------

    def read_state(self) -> Optional[dict]:
        """Read the full state dump (register 0xFF, §7).

        Returns a dict with all 40 fields parsed from the 139-byte payload,
        or None if the read fails.
        """
        resp = self._send_and_recv(CMD_READ, REG_ALL, b"\x00", wait=1.0)

        # The full dump is large; collect any additional bytes
        time.sleep(0.3)
        extra = self._ser.read(self._ser.in_waiting or 0)
        if extra:
            resp = resp + extra

        pkt = self._find_response(resp, REG_ALL)
        if not pkt or len(pkt) < 20:
            return None

        d = pkt[4:-1]  # payload (strip header bytes and checksum)
        if len(d) < 139:
            return None

        prot_code = d[108]
        state = {
            "input_voltage": parse_float(d, 0),
            "voltage_setpoint": parse_float(d, 4),
            "current_setpoint": parse_float(d, 8),
            "output_voltage": parse_float(d, 12),
            "output_current": parse_float(d, 16),
            "output_power": parse_float(d, 20),
            "temperature": parse_float(d, 24),
            "presets": [
                {
                    "voltage": parse_float(d, 28 + i * 8),
                    "current": parse_float(d, 32 + i * 8),
                }
                for i in range(6)
            ],
            "ovp": parse_float(d, 76),
            "ocp": parse_float(d, 80),
            "opp": parse_float(d, 84),
            "otp": parse_float(d, 88),
            "lvp": parse_float(d, 92),
            "brightness": d[96],
            "volume": d[97],
            "metering": "running" if d[98] == 0 else "stopped",
            "ah_counter": parse_float(d, 99),
            "wh_counter": parse_float(d, 103),
            "output_on": bool(d[107]),
            "protection_code": prot_code,
            "protection": (
                PROTECTION_NAMES[prot_code]
                if prot_code < len(PROTECTION_NAMES)
                else f"Unknown({prot_code})"
            ),
            "mode": "CV" if d[109] else "CC",
            "max_voltage": parse_float(d, 111),
            "max_current": parse_float(d, 115),
            "ovp_ceiling": parse_float(d, 119),
            "ocp_ceiling": parse_float(d, 123),
            "opp_ceiling": parse_float(d, 127),
            "otp_ceiling": parse_float(d, 131),
            "lvp_ceiling": parse_float(d, 135),
        }
        return state

    def read_voltage(self) -> float:
        """Read the measured output voltage."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["output_voltage"]

    def read_current(self) -> float:
        """Read the measured output current."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["output_current"]

    def read_power(self) -> float:
        """Read the measured output power."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["output_power"]

    def read_input_voltage(self) -> float:
        """Read the input supply voltage."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["input_voltage"]

    def read_temperature(self) -> float:
        """Read the internal temperature in degrees Celsius."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["temperature"]

    # -- Output control (§5.2) -----------------------------------------------

    def set_voltage(self, volts: float):
        """Set the voltage setpoint. Validates against device max."""
        if volts < 0 or volts > self._max_voltage:
            raise ValueError(
                f"Voltage {volts:.3f}V out of range [0, {self._max_voltage:.1f}V]"
            )
        self._write_float(REG_W_VOLTAGE, volts)

    def set_current(self, amps: float):
        """Set the current limit. Validates against device max."""
        if amps < 0 or amps > self._max_current:
            raise ValueError(
                f"Current {amps:.3f}A out of range [0, {self._max_current:.1f}A]"
            )
        self._write_float(REG_W_CURRENT, amps)

    def output_on(self):
        """Enable the output."""
        self._write_byte(REG_W_OUTPUT, 1)

    def output_off(self):
        """Disable the output."""
        self._write_byte(REG_W_OUTPUT, 0)

    def set_output(self, volts: float, amps: float):
        """Set voltage and current, then enable output.

        Always sets V and A *before* enabling to prevent transients.
        """
        self.set_voltage(volts)
        self.set_current(amps)
        self.output_on()

    # -- Protection thresholds (§5.2, validated against ceilings d36-d40) ----

    def set_ovp(self, volts: float):
        """Set over-voltage protection threshold."""
        if self._ovp_ceiling > 0 and volts > self._ovp_ceiling:
            raise ValueError(
                f"OVP {volts:.2f}V exceeds ceiling {self._ovp_ceiling:.2f}V"
            )
        if volts < 0:
            raise ValueError("OVP must be non-negative")
        self._write_float(REG_W_OVP, volts)

    def set_ocp(self, amps: float):
        """Set over-current protection threshold."""
        if self._ocp_ceiling > 0 and amps > self._ocp_ceiling:
            raise ValueError(
                f"OCP {amps:.3f}A exceeds ceiling {self._ocp_ceiling:.3f}A"
            )
        if amps < 0:
            raise ValueError("OCP must be non-negative")
        self._write_float(REG_W_OCP, amps)

    def set_opp(self, watts: float):
        """Set over-power protection threshold."""
        if self._opp_ceiling > 0 and watts > self._opp_ceiling:
            raise ValueError(
                f"OPP {watts:.2f}W exceeds ceiling {self._opp_ceiling:.2f}W"
            )
        if watts < 0:
            raise ValueError("OPP must be non-negative")
        self._write_float(REG_W_OPP, watts)

    def set_otp(self, celsius: float):
        """Set over-temperature protection threshold."""
        if self._otp_ceiling > 0 and celsius > self._otp_ceiling:
            raise ValueError(
                f"OTP {celsius:.1f}°C exceeds ceiling {self._otp_ceiling:.1f}°C"
            )
        if celsius < 0:
            raise ValueError("OTP must be non-negative")
        self._write_float(REG_W_OTP, celsius)

    def set_lvp(self, volts: float):
        """Set low-voltage protection threshold."""
        if self._lvp_ceiling > 0 and volts > self._lvp_ceiling:
            raise ValueError(
                f"LVP {volts:.2f}V exceeds ceiling {self._lvp_ceiling:.2f}V"
            )
        if volts < 0:
            raise ValueError("LVP must be non-negative")
        self._write_float(REG_W_LVP, volts)

    # -- Presets (§5.3) ------------------------------------------------------

    def set_preset(self, n: int, volts: float, amps: float):
        """Set preset M1-M6 (n=1..6)."""
        if n < 1 or n > 6:
            raise ValueError(f"Preset number must be 1-6, got {n}")
        if volts < 0 or volts > self._max_voltage:
            raise ValueError(
                f"Preset voltage {volts:.3f}V out of range [0, {self._max_voltage:.1f}V]"
            )
        if amps < 0 or amps > self._max_current:
            raise ValueError(
                f"Preset current {amps:.3f}A out of range [0, {self._max_current:.1f}A]"
            )
        v_reg, i_reg = PRESET_REGS[n - 1]
        self._write_float(v_reg, volts)
        self._write_float(i_reg, amps)

    def get_presets(self) -> list[dict]:
        """Read all 6 presets from the device state."""
        state = self.read_state()
        if state is None:
            raise IOError("Failed to read state from device")
        return state["presets"]

    # -- Utilities (§5.2) ----------------------------------------------------

    def set_brightness(self, level: int):
        """Set display brightness (0-255)."""
        if level < 0 or level > 255:
            raise ValueError(f"Brightness must be 0-255, got {level}")
        self._write_byte(REG_W_BRIGHTNESS, level)

    def set_volume(self, level: int):
        """Set beep volume level."""
        if level < 0 or level > 255:
            raise ValueError(f"Volume must be 0-255, got {level}")
        self._write_byte(REG_W_VOLUME, level)

    def start_metering(self):
        """Enable the Ah/Wh metering counters."""
        self._write_byte(REG_W_METERING, 1)

    def stop_metering(self):
        """Disable the Ah/Wh metering counters."""
        self._write_byte(REG_W_METERING, 0)

    def restart(self):
        """Send a system restart command (§3, CMD_RESTART)."""
        self._send(CMD_RESTART, 0x00, b"\x01")

    # -- Sweeps (blocking) ---------------------------------------------------

    def sweep_voltage(
        self,
        start_v: float,
        stop_v: float,
        step_v: float,
        hold_s: float,
        current_limit: float,
        callback: Optional[Callable] = None,
    ) -> list[dict]:
        """Sweep voltage from start to stop, reading measurements at each step.

        This is a PC-side automation feature (§11, "Current Scan"). The output is
        enabled at the start and disabled at the end.

        Args:
            start_v: Starting voltage
            stop_v: Ending voltage
            step_v: Voltage increment per step (must be positive)
            hold_s: Dwell time at each step in seconds
            current_limit: Fixed current limit during sweep
            callback: Optional function called at each step with
                      (step_index, voltage, current, power). Return False to abort.

        Returns:
            List of dicts with keys: step, voltage, current, power
        """
        if step_v <= 0:
            raise ValueError("step_v must be positive")
        if start_v < 0 or start_v > self._max_voltage:
            raise ValueError(f"start_v {start_v}V out of range [0, {self._max_voltage}V]")
        if stop_v < 0 or stop_v > self._max_voltage:
            raise ValueError(f"stop_v {stop_v}V out of range [0, {self._max_voltage}V]")
        if current_limit < 0 or current_limit > self._max_current:
            raise ValueError(
                f"current_limit {current_limit}A out of range [0, {self._max_current}A]"
            )

        self.set_current(current_limit)
        self.set_voltage(start_v)
        self.output_on()

        results = []
        step_idx = 0
        v = start_v
        direction = 1 if stop_v >= start_v else -1

        try:
            while (direction > 0 and v <= stop_v + step_v / 100) or \
                  (direction < 0 and v >= stop_v - step_v / 100):
                clamped = max(0.0, min(v, self._max_voltage))
                self.set_voltage(clamped)
                time.sleep(hold_s)

                state = self.read_state()
                if state:
                    reading = {
                        "step": step_idx,
                        "voltage": state["output_voltage"],
                        "current": state["output_current"],
                        "power": state["output_power"],
                    }
                    results.append(reading)

                    if callback is not None:
                        ret = callback(
                            step_idx,
                            state["output_voltage"],
                            state["output_current"],
                            state["output_power"],
                        )
                        if ret is False:
                            break

                step_idx += 1
                v += step_v * direction
        finally:
            self.output_off()

        return results

    def sweep_current(
        self,
        start_a: float,
        stop_a: float,
        step_a: float,
        hold_s: float,
        voltage: float,
        callback: Optional[Callable] = None,
    ) -> list[dict]:
        """Sweep current from start to stop, reading measurements at each step.

        This is a PC-side automation feature (§11, "Voltage Scan"). The output is
        enabled at the start and disabled at the end.

        Args:
            start_a: Starting current
            stop_a: Ending current
            step_a: Current increment per step (must be positive)
            hold_s: Dwell time at each step in seconds
            voltage: Fixed voltage during sweep
            callback: Optional function called at each step with
                      (step_index, voltage, current, power). Return False to abort.

        Returns:
            List of dicts with keys: step, voltage, current, power
        """
        if step_a <= 0:
            raise ValueError("step_a must be positive")
        if start_a < 0 or start_a > self._max_current:
            raise ValueError(f"start_a {start_a}A out of range [0, {self._max_current}A]")
        if stop_a < 0 or stop_a > self._max_current:
            raise ValueError(f"stop_a {stop_a}A out of range [0, {self._max_current}A]")
        if voltage < 0 or voltage > self._max_voltage:
            raise ValueError(
                f"voltage {voltage}V out of range [0, {self._max_voltage}V]"
            )

        self.set_voltage(voltage)
        self.set_current(start_a)
        self.output_on()

        results = []
        step_idx = 0
        a = start_a
        direction = 1 if stop_a >= start_a else -1

        try:
            while (direction > 0 and a <= stop_a + step_a / 100) or \
                  (direction < 0 and a >= stop_a - step_a / 100):
                clamped = max(0.0, min(a, self._max_current))
                self.set_current(clamped)
                time.sleep(hold_s)

                state = self.read_state()
                if state:
                    reading = {
                        "step": step_idx,
                        "voltage": state["output_voltage"],
                        "current": state["output_current"],
                        "power": state["output_power"],
                    }
                    results.append(reading)

                    if callback is not None:
                        ret = callback(
                            step_idx,
                            state["output_voltage"],
                            state["output_current"],
                            state["output_power"],
                        )
                        if ret is False:
                            break

                step_idx += 1
                a += step_a * direction
        finally:
            self.output_off()

        return results

    # -- Telemetry (background thread) (§4.4) --------------------------------

    def start_telemetry(self, callback: Callable):
        """Start a background thread that reads auto-pushed telemetry.

        The callback receives a dict with parsed telemetry data. The dict keys
        depend on which register the device pushed:
        - 0xC0: {"input_voltage": float}
        - 0xC3: {"output_voltage": float, "output_current": float, "output_power": float}
        - 0xC4: {"temperature": float}
        - 0xDB: {"output_on": bool}
        - 0xDC: {"protection_code": int, "protection": str}
        - 0xDD: {"mode": str}
        - 0xE2: {"max_voltage": float}
        - 0xE3: {"max_current": float}
        """
        self.stop_telemetry()
        self._telemetry_callback = callback
        self._telemetry_stop.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True
        )
        self._telemetry_thread.start()

    def stop_telemetry(self):
        """Stop the telemetry background thread."""
        if self._telemetry_thread is not None:
            self._telemetry_stop.set()
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
            self._telemetry_callback = None

    def _telemetry_loop(self):
        """Background loop that reads and dispatches telemetry packets."""
        while not self._telemetry_stop.is_set():
            try:
                with self._telemetry_lock:
                    if self._ser and self._ser.is_open and self._ser.in_waiting:
                        data = self._ser.read(self._ser.in_waiting)
                    else:
                        data = b""
            except Exception:
                break

            if data:
                for pkt in parse_responses(data):
                    parsed = self._parse_telemetry_packet(pkt)
                    if parsed and self._telemetry_callback:
                        try:
                            self._telemetry_callback(parsed)
                        except Exception:
                            pass

            self._telemetry_stop.wait(0.05)

    @staticmethod
    def _parse_telemetry_packet(pkt: bytes) -> Optional[dict]:
        """Parse a single telemetry packet into a dict."""
        if len(pkt) < 5:
            return None
        reg = pkt[2]
        payload = pkt[4:-1]

        if reg == REG_INPUT_VOLTAGE and len(payload) >= 4:
            return {"input_voltage": parse_float(payload)}
        elif reg == REG_OUTPUT_VIP and len(payload) >= 12:
            return {
                "output_voltage": parse_float(payload, 0),
                "output_current": parse_float(payload, 4),
                "output_power": parse_float(payload, 8),
            }
        elif reg == REG_TEMPERATURE and len(payload) >= 4:
            return {"temperature": parse_float(payload)}
        elif reg == REG_OUTPUT_STATE and len(payload) >= 1:
            return {"output_on": bool(payload[0])}
        elif reg == REG_PROTECTION and len(payload) >= 1:
            code = payload[0]
            return {
                "protection_code": code,
                "protection": (
                    PROTECTION_NAMES[code]
                    if code < len(PROTECTION_NAMES)
                    else f"Unknown({code})"
                ),
            }
        elif reg == REG_CVCC_MODE and len(payload) >= 1:
            return {"mode": "CV" if payload[0] else "CC"}
        elif reg == REG_MAX_VOLTAGE and len(payload) >= 4:
            return {"max_voltage": parse_float(payload)}
        elif reg == REG_MAX_CURRENT and len(payload) >= 4:
            return {"max_current": parse_float(payload)}
        return None


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _cli():
    import argparse
    import json as _json
    import sys

    parser = argparse.ArgumentParser(
        prog="dps150",
        description="FNIRSI DPS-150 command-line interface",
    )
    parser.add_argument(
        "-p", "--port",
        default="/dev/cu.usbmodem065AD9D205B31",
        help="serial port (default: %(default)s)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # -- info ----------------------------------------------------------------
    sub.add_parser("info", help="show device info")

    # -- state ---------------------------------------------------------------
    sub.add_parser("state", help="read full state dump (JSON)")

    # -- voltage / current readings ------------------------------------------
    sub.add_parser("voltage", help="read measured output voltage")
    sub.add_parser("current", help="read measured output current")
    sub.add_parser("power", help="read measured output power")
    sub.add_parser("input-voltage", help="read input supply voltage")
    sub.add_parser("temperature", help="read internal temperature")

    # -- set-voltage ---------------------------------------------------------
    p = sub.add_parser("set-voltage", help="set voltage setpoint")
    p.add_argument("volts", type=float)

    # -- set-current ---------------------------------------------------------
    p = sub.add_parser("set-current", help="set current limit")
    p.add_argument("amps", type=float)

    # -- set-output ----------------------------------------------------------
    p = sub.add_parser("set-output", help="set V/A and enable output")
    p.add_argument("volts", type=float)
    p.add_argument("amps", type=float)

    # -- on / off ------------------------------------------------------------
    sub.add_parser("on", help="enable output")
    sub.add_parser("off", help="disable output")

    # -- protection ----------------------------------------------------------
    p = sub.add_parser("set-ovp", help="set over-voltage protection")
    p.add_argument("volts", type=float)

    p = sub.add_parser("set-ocp", help="set over-current protection")
    p.add_argument("amps", type=float)

    p = sub.add_parser("set-opp", help="set over-power protection")
    p.add_argument("watts", type=float)

    p = sub.add_parser("set-otp", help="set over-temperature protection")
    p.add_argument("celsius", type=float)

    p = sub.add_parser("set-lvp", help="set low-voltage protection")
    p.add_argument("volts", type=float)

    # -- presets -------------------------------------------------------------
    sub.add_parser("presets", help="read all presets")

    p = sub.add_parser("set-preset", help="set a preset (M1-M6)")
    p.add_argument("n", type=int, choices=range(1, 7), metavar="N")
    p.add_argument("volts", type=float)
    p.add_argument("amps", type=float)

    # -- brightness / volume -------------------------------------------------
    p = sub.add_parser("set-brightness", help="set display brightness (0-255)")
    p.add_argument("level", type=int)

    p = sub.add_parser("set-volume", help="set beep volume (0-255)")
    p.add_argument("level", type=int)

    # -- metering ------------------------------------------------------------
    sub.add_parser("start-metering", help="enable Ah/Wh counters")
    sub.add_parser("stop-metering", help="disable Ah/Wh counters")

    # -- sweeps --------------------------------------------------------------
    p = sub.add_parser("sweep-voltage", help="voltage sweep with readings")
    p.add_argument("start_v", type=float)
    p.add_argument("stop_v", type=float)
    p.add_argument("step_v", type=float)
    p.add_argument("hold_s", type=float)
    p.add_argument("current_limit", type=float)

    p = sub.add_parser("sweep-current", help="current sweep with readings")
    p.add_argument("start_a", type=float)
    p.add_argument("stop_a", type=float)
    p.add_argument("step_a", type=float)
    p.add_argument("hold_s", type=float)
    p.add_argument("voltage", type=float)

    # -- restart -------------------------------------------------------------
    sub.add_parser("restart", help="restart the device")

    args = parser.parse_args()

    # Commands that should leave output running when done
    KEEP_OUTPUT = {"set-voltage", "set-current", "set-output", "on",
                   "set-ovp", "set-ocp", "set-opp", "set-otp", "set-lvp",
                   "set-preset", "set-brightness", "set-volume",
                   "start-metering", "stop-metering"}

    psu = DPS150(args.port)
    psu.connect()

    try:
        cmd = args.command

        if cmd == "info":
            print(f"Model:    {psu.model}")
            print(f"Firmware: {psu.firmware_version}")
            print(f"Hardware: {psu.hardware_version}")
            print(f"Max V:    {psu.max_voltage:.1f} V")
            print(f"Max A:    {psu.max_current:.1f} A")

        elif cmd == "state":
            state = psu.read_state()
            if state:
                print(_json.dumps(state, indent=2))
            else:
                print("Error: failed to read state", file=sys.stderr)
                sys.exit(1)

        elif cmd == "voltage":
            print(f"{psu.read_voltage():.3f}")
        elif cmd == "current":
            print(f"{psu.read_current():.4f}")
        elif cmd == "power":
            print(f"{psu.read_power():.3f}")
        elif cmd == "input-voltage":
            print(f"{psu.read_input_voltage():.2f}")
        elif cmd == "temperature":
            print(f"{psu.read_temperature():.1f}")

        elif cmd == "set-voltage":
            psu.set_voltage(args.volts)
            print(f"Voltage setpoint: {args.volts:.3f} V")
        elif cmd == "set-current":
            psu.set_current(args.amps)
            print(f"Current limit: {args.amps:.3f} A")
        elif cmd == "set-output":
            psu.set_output(args.volts, args.amps)
            print(f"Output ON: {args.volts:.3f} V / {args.amps:.3f} A")
        elif cmd == "on":
            psu.output_on()
            print("Output ON")
        elif cmd == "off":
            psu.output_off()
            print("Output OFF")

        elif cmd == "set-ovp":
            psu.set_ovp(args.volts)
            print(f"OVP: {args.volts:.2f} V")
        elif cmd == "set-ocp":
            psu.set_ocp(args.amps)
            print(f"OCP: {args.amps:.3f} A")
        elif cmd == "set-opp":
            psu.set_opp(args.watts)
            print(f"OPP: {args.watts:.2f} W")
        elif cmd == "set-otp":
            psu.set_otp(args.celsius)
            print(f"OTP: {args.celsius:.1f} C")
        elif cmd == "set-lvp":
            psu.set_lvp(args.volts)
            print(f"LVP: {args.volts:.2f} V")

        elif cmd == "presets":
            presets = psu.get_presets()
            for i, p in enumerate(presets):
                print(f"M{i+1}: {p['voltage']:.2f} V / {p['current']:.3f} A")
        elif cmd == "set-preset":
            psu.set_preset(args.n, args.volts, args.amps)
            print(f"M{args.n}: {args.volts:.2f} V / {args.amps:.3f} A")

        elif cmd == "set-brightness":
            psu.set_brightness(args.level)
            print(f"Brightness: {args.level}")
        elif cmd == "set-volume":
            psu.set_volume(args.level)
            print(f"Volume: {args.level}")

        elif cmd == "start-metering":
            psu.start_metering()
            print("Metering started")
        elif cmd == "stop-metering":
            psu.stop_metering()
            print("Metering stopped")

        elif cmd == "sweep-voltage":
            def _sv_cb(i, v, a, p):
                print(f"  [{i:3d}] {v:7.3f} V  {a:6.4f} A  {p:7.3f} W")
            results = psu.sweep_voltage(
                args.start_v, args.stop_v, args.step_v,
                args.hold_s, args.current_limit, callback=_sv_cb,
            )
            print(f"\n{len(results)} steps recorded")

        elif cmd == "sweep-current":
            def _sc_cb(i, v, a, p):
                print(f"  [{i:3d}] {v:7.3f} V  {a:6.4f} A  {p:7.3f} W")
            results = psu.sweep_current(
                args.start_a, args.stop_a, args.step_a,
                args.hold_s, args.voltage, callback=_sc_cb,
            )
            print(f"\n{len(results)} steps recorded")

        elif cmd == "restart":
            psu.restart()
            print("Restart command sent")

    except (ValueError, IOError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if args.command in KEEP_OUTPUT:
            psu.close()
        else:
            psu.disconnect()


if __name__ == "__main__":
    _cli()
