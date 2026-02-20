"""Packet-level protocol tests -- no mocking needed.

Verifies build_packet, parse_responses, parse_float, encode_float against
the concrete examples in PROTOCOL.md section 9.
"""

import struct
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from dps150 import build_packet, parse_responses, parse_float, encode_float


# ---------------------------------------------------------------------------
# build_packet -- section 9 concrete examples
# ---------------------------------------------------------------------------

class TestBuildPacket:

    def test_session_open(self):
        """Section 9: Session open -> F1 C1 00 01 01 02"""
        pkt = build_packet(0xF1, 0xC1, 0x00, b"\x01")
        assert pkt == bytes([0xF1, 0xC1, 0x00, 0x01, 0x01, 0x02])

    def test_session_close(self):
        """Section 9: Session close -> F1 C1 00 01 00 01"""
        pkt = build_packet(0xF1, 0xC1, 0x00, b"\x00")
        assert pkt == bytes([0xF1, 0xC1, 0x00, 0x01, 0x00, 0x01])

    def test_set_voltage_5v(self):
        """Section 9: Set voltage 5.0V -> F1 B1 C1 04 00 00 A0 40 A5"""
        data = struct.pack("<f", 5.0)
        pkt = build_packet(0xF1, 0xB1, 0xC1, data)
        assert pkt == bytes([0xF1, 0xB1, 0xC1, 0x04, 0x00, 0x00, 0xA0, 0x40, 0xA5])

    def test_set_current_1a(self):
        """Section 9: Set current 1.0A -> F1 B1 C2 04 00 00 80 3F 85"""
        data = struct.pack("<f", 1.0)
        pkt = build_packet(0xF1, 0xB1, 0xC2, data)
        assert pkt == bytes([0xF1, 0xB1, 0xC2, 0x04, 0x00, 0x00, 0x80, 0x3F, 0x85])

    def test_output_on(self):
        """Section 9: Output ON -> F1 B1 DB 01 01 DD"""
        pkt = build_packet(0xF1, 0xB1, 0xDB, b"\x01")
        assert pkt == bytes([0xF1, 0xB1, 0xDB, 0x01, 0x01, 0xDD])

    def test_output_off(self):
        """Section 9: Output OFF -> F1 B1 DB 01 00 DC"""
        pkt = build_packet(0xF1, 0xB1, 0xDB, b"\x00")
        assert pkt == bytes([0xF1, 0xB1, 0xDB, 0x01, 0x00, 0xDC])

    def test_read_all_registers(self):
        """Section 9: Read all -> F1 A1 FF 01 00 00"""
        pkt = build_packet(0xF1, 0xA1, 0xFF, b"\x00")
        assert pkt == bytes([0xF1, 0xA1, 0xFF, 0x01, 0x00, 0x00])

    def test_set_baud_115200(self):
        """Section 9: Set baud -> F1 B0 00 01 05 06"""
        pkt = build_packet(0xF1, 0xB0, 0x00, b"\x05")
        assert pkt == bytes([0xF1, 0xB0, 0x00, 0x01, 0x05, 0x06])

    def test_empty_data(self):
        """Packet with no data bytes."""
        pkt = build_packet(0xF1, 0xA1, 0xDE, b"")
        # checksum = (0xDE + 0) & 0xFF = 0xDE
        assert pkt == bytes([0xF1, 0xA1, 0xDE, 0x00, 0xDE])


# ---------------------------------------------------------------------------
# parse_responses
# ---------------------------------------------------------------------------

class TestParseResponses:

    def test_single_packet(self):
        """Parse a single complete packet."""
        pkt = bytes([0xF0, 0xA1, 0xDB, 0x01, 0x01, 0xDD])
        result = parse_responses(pkt)
        assert len(result) == 1
        assert result[0] == pkt

    def test_two_concatenated(self):
        """Parse two packets concatenated in one buffer."""
        pkt1 = bytes([0xF0, 0xA1, 0xDB, 0x01, 0x01, 0xDD])
        pkt2 = bytes([0xF0, 0xA1, 0xDC, 0x01, 0x00, 0xDD])
        result = parse_responses(pkt1 + pkt2)
        assert len(result) == 2
        assert result[0] == pkt1
        assert result[1] == pkt2

    def test_truncated_buffer(self):
        """Truncated buffer returns only complete packets."""
        pkt = bytes([0xF0, 0xA1, 0xDB, 0x01, 0x01, 0xDD])
        truncated = pkt + bytes([0xF0, 0xA1, 0xDC])  # incomplete second packet
        result = parse_responses(truncated)
        assert len(result) == 1
        assert result[0] == pkt

    def test_empty_buffer(self):
        assert parse_responses(b"") == []

    def test_too_short(self):
        """Buffer shorter than 4 bytes yields nothing."""
        assert parse_responses(bytes([0xF0, 0xA1, 0xDB])) == []

    def test_length_exceeds_buffer(self):
        """Packet whose declared length exceeds available bytes."""
        buf = bytes([0xF0, 0xA1, 0xDB, 0x05, 0x01])  # claims 5 data bytes, only 1
        assert parse_responses(buf) == []


# ---------------------------------------------------------------------------
# parse_float / encode_float
# ---------------------------------------------------------------------------

class TestFloatRoundTrip:

    def test_5_0(self):
        encoded = encode_float(5.0)
        assert parse_float(encoded) == 5.0

    def test_1_0(self):
        encoded = encode_float(1.0)
        assert parse_float(encoded) == 1.0

    def test_0_0(self):
        encoded = encode_float(0.0)
        assert parse_float(encoded) == 0.0

    def test_24_0(self):
        encoded = encode_float(24.0)
        assert parse_float(encoded) == 24.0

    def test_parse_with_offset(self):
        """parse_float with a non-zero offset."""
        buf = b"\x00\x00\x00\x00" + encode_float(3.14)
        val = parse_float(buf, offset=4)
        assert abs(val - 3.14) < 1e-5

    def test_encode_format(self):
        """encode_float returns 4 bytes in IEEE 754 LE."""
        data = encode_float(5.0)
        assert len(data) == 4
        assert data == bytes([0x00, 0x00, 0xA0, 0x40])
