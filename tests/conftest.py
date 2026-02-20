"""Shared fixtures for DPS-150 tests."""

import struct
from unittest.mock import MagicMock, patch

import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dps150 import DPS150, build_packet, HEADER_RX, CMD_READ, REG_ALL


@pytest.fixture
def fake_state_payload():
    """Build a 139-byte payload with known values.

    Values:
        input_voltage=12.0, voltage_setpoint=5.0, current_setpoint=1.0,
        output_voltage=4.99, output_current=0.5, output_power=2.495,
        temperature=35.0, 6 presets all 5V/1A,
        ovp=25.0, ocp=5.5, opp=130.0, otp=80.0, lvp=3.0,
        brightness=128, volume=64, metering=stopped(1),
        ah_counter=0.0, wh_counter=0.0,
        output_on=True, protection_code=0(OK), mode=CV(1),
        unknown_byte=0,
        max_voltage=24.0, max_current=5.0,
        ovp_ceiling=25.0, ocp_ceiling=5.5, opp_ceiling=130.0,
        otp_ceiling=80.0, lvp_ceiling=3.0
    """
    d = bytearray(139)
    offset = 0

    def put_float(val):
        nonlocal offset
        struct.pack_into("<f", d, offset, val)
        offset += 4

    # d1-d7: input_voltage through temperature
    put_float(12.0)   # input voltage
    put_float(5.0)    # voltage setpoint
    put_float(1.0)    # current setpoint
    put_float(4.99)   # output voltage
    put_float(0.5)    # output current
    put_float(2.495)  # output power
    put_float(35.0)   # temperature

    # d8-d19: 6 presets (each voltage + current)
    for _ in range(6):
        put_float(5.0)   # preset voltage
        put_float(1.0)   # preset current

    # d20-d24: protection thresholds
    put_float(25.0)   # ovp
    put_float(5.5)    # ocp
    put_float(130.0)  # opp
    put_float(80.0)   # otp
    put_float(3.0)    # lvp

    # d25-d27: brightness, volume, metering (single bytes)
    d[96] = 128       # brightness
    d[97] = 64        # volume
    d[98] = 1         # metering stopped
    offset = 99

    # d28-d29: ah/wh counters
    put_float(0.0)    # ah_counter
    put_float(0.0)    # wh_counter

    # d30-d33: output state, protection, mode, unknown
    d[107] = 1        # output on
    d[108] = 0        # protection OK
    d[109] = 1        # CV mode
    d[110] = 0        # unknown byte
    offset = 111

    # d34-d35: max voltage/current
    put_float(24.0)   # max voltage
    put_float(5.0)    # max current

    # d36-d40: protection ceilings
    put_float(25.0)   # ovp ceiling
    put_float(5.5)    # ocp ceiling
    put_float(130.0)  # opp ceiling
    put_float(80.0)   # otp ceiling
    put_float(3.0)    # lvp ceiling

    return bytes(d)


@pytest.fixture
def fake_state_packet(fake_state_payload):
    """Wrap fake_state_payload in a valid F0 A1 FF <len> <payload> <checksum> packet."""
    header = 0xF0
    command = 0xA1
    register = 0xFF
    length = len(fake_state_payload)
    checksum = (register + length + sum(fake_state_payload)) & 0xFF
    pkt = bytes([header, command, register, length]) + fake_state_payload + bytes([checksum])
    return pkt


@pytest.fixture
def mock_psu(fake_state_packet):
    """A DPS150 instance with serial fully mocked out.

    - _ser is a MagicMock
    - max values and ceilings are pre-loaded
    - _send, _send_and_recv, _drain are stubbed
    - _send_and_recv returns the fake state packet when register is 0xFF
    """
    with patch("dps150.serial.Serial"):
        psu = DPS150("/dev/fake", baud=115200)

    psu._ser = MagicMock()
    psu._ser.is_open = True
    psu._ser.in_waiting = 0

    # Pre-load device capabilities
    psu._model = "DPS-150"
    psu._fw_version = "1.0.0"
    psu._hw_version = "2.0"
    psu._max_voltage = 24.0
    psu._max_current = 5.0
    psu._ovp_ceiling = 25.0
    psu._ocp_ceiling = 5.5
    psu._opp_ceiling = 130.0
    psu._otp_ceiling = 80.0
    psu._lvp_ceiling = 3.0

    # Stub low-level I/O
    psu._send = MagicMock()
    psu._drain = MagicMock(return_value=b"")

    def fake_send_and_recv(command, register, data=b"", wait=0.2):
        if register == 0xFF:
            return fake_state_packet
        return b""

    psu._send_and_recv = MagicMock(side_effect=fake_send_and_recv)

    return psu
