# FNIRSI DPS-150 Serial Protocol Specification

> Reverse-engineered from the official FNIRSI Windows app (`FNIRSI Power supply.exe`, .NET assembly)
> and cross-referenced against 4 independent open-source implementations:
> [cho45/fnirsi-dps-150](https://github.com/cho45/fnirsi-dps-150) (JS),
> [svenk123/dps150tool](https://github.com/svenk123/dps150tool) (C),
> [KochC/DPS-150-python-library](https://github.com/KochC/DPS-150-python-library) (Python),
> [huiminghao/fnirsi-dps-150-rs](https://github.com/huiminghao/fnirsi-dps-150-rs) (Rust).

---

## 1. Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | USB CDC (Virtual COM Port) |
| USB VID:PID | `0x2E3C:0x5740` |
| Baud rates | 9600, 19200, 38400, 57600, **115200** (default) |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | RTS asserted (see note) |
| Read timeout | 500ms (official app) |

**Baud rate index mapping:**

| Index | Baud Rate |
|-------|-----------|
| 0 | AUTO (reserved, not used by official app) |
| 1 | 9600 |
| 2 | 19200 |
| 3 | 38400 |
| 4 | 57600 |
| 5 | 115200 |

> **Note on RTS:** The official app sets `RtsEnable = true` which asserts the RTS line but does
> NOT enable full RTS/CTS hardware flow control. The device may require RTS to be asserted before
> it will send data. cho45's WebSerial implementation uses `flowControl: 'hardware'` (full RTS/CTS)
> which also works. When in doubt, assert RTS.

> **Note:** The index sent over the wire is 1-based. Some implementations use a 0-based array
> internally and add 1 before sending (e.g. array index 4 + 1 = wire value 5 for 115200).
> The svenk123 C tool sends index 4 directly — this may be a bug in that implementation,
> or the device may accept both. Use index **5** for 115200 to match the official app.

---

## 2. Packet Structure

Every packet (TX and RX) follows the same format:

```
┌────────┬─────────┬──────────┬────────┬──────────────┬──────────┐
│ Header │ Command │ Register │ Length │ Data (0..N)  │ Checksum │
│ 1 byte │ 1 byte  │ 1 byte   │ 1 byte │ N bytes      │ 1 byte   │
└────────┴─────────┴──────────┴────────┴──────────────┴──────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| Header (c1) | 1 | `0xF1` for host→device, `0xF0` for device→host |
| Command (c2) | 1 | Command category (see below) |
| Register (c3) | 1 | Target register address |
| Length (c4) | 1 | Number of data bytes that follow |
| Data (c5) | N | Payload (variable length, can be 0) |
| Checksum (c6) | 1 | `(c3 + c4 + sum(c5[])) & 0xFF` |

**Total packet size** = 5 + Length

### Checksum Algorithm

```
checksum = register + length
for each byte in data:
    checksum = (checksum + byte) & 0xFF
```

> **Note:** The header (c1) and command (c2) bytes are NOT included in the checksum.

---

## 3. Command Categories (c2)

| Byte | Hex | Direction | Purpose |
|------|-----|-----------|---------|
| 161 | 0xA1 | Read | Read register value / response from device |
| 176 | 0xB0 | Write | Set baud rate |
| 177 | 0xB1 | Write | Write register value |
| 192 | 0xC0 | Write | Enter firmware upgrade (DFU) mode — **not** a normal restart (see §12) |
| 193 | 0xC1 | Write | Session control (connect/disconnect) |

---

## 4. Connection Lifecycle

### 4.1 Connect (Session Open)

```
TX: F1 C1 00 01 01 02
```

After opening the serial port, send this handshake.

The official app then reads register 0xE1 to verify the device address:

```
TX: F1 A1 E1 01 00 E2    (read device address register)
```

The device responds with its address byte (configurable, range 1-255). The official app compares this against the user-selected device address ("设备地址"). If they match, `IsSpecific` is set to true and initialization proceeds. The app retries up to **2 times** with a **1-second delay** between attempts before giving up.

Most third-party implementations skip this verification step entirely and it works fine.

### 4.2 Initialization Sequence

After successful handshake, send these commands in order:

1. **Set baud rate**: `F1 B0 00 01 05 06` (05 = 115200)
2. **Read model name**: `F1 A1 DE 01 00 DF`
3. **Read firmware version**: `F1 A1 E0 01 00 E1`
4. **Read hardware version**: `F1 A1 DF 01 00 E0`
5. **Read all registers**: `F1 A1 FF 01 00 00`

> **Note:** The official app sends firmware version (0xE0) before hardware version (0xDF).
> The order doesn't matter functionally — they are independent reads. A 1-second timer
> retries the model name and firmware version reads until responses are received.

### 4.3 Disconnect (Session Close)

```
TX: F1 C1 00 01 00 01
```

### 4.4 Telemetry

Once connected, the device automatically pushes telemetry data. There are two categories:

**Periodic telemetry** (sent continuously, even when output is stopped):

| Register | Hex | Description |
|----------|-----|-------------|
| 192 | 0xC0 | Input voltage |
| 195 | 0xC3 | Output V/I/P (3×float32, ~every 500ms) |
| 196 | 0xC4 | Temperature |
| 226 | 0xE2 | Max voltage capability |
| 227 | 0xE3 | Max current capability |

**On-change telemetry** (sent when state changes):

| Register | Hex | Description |
|----------|-----|-------------|
| 219 | 0xDB | Output state changed |
| 220 | 0xDC | Protection status changed |
| 221 | 0xDD | CV/CC mode changed |

**Only during RUN** (sent only when output is enabled):

| Register | Hex | Description |
|----------|-----|-------------|
| 217 | 0xD9 | Ah counter (capacity) |
| 218 | 0xDA | Wh counter (energy) |

You can also poll explicitly by sending read-all (`F1 A1 FF 01 00 00`).

### 4.5 Inter-Command Delay

A ~50ms delay between consecutive commands is recommended for reliable communication. The svenk123 C implementation uses `usleep(50000)` after each write.

---

## 5. Complete Register Map

### 5.1 Readable Registers (c2 = 0xA1)

| Register | Hex | Type | Description |
|----------|-----|------|-------------|
| 192 | 0xC0 | float32 | Input voltage reading |
| 193 | 0xC1 | float32 | Voltage set-point |
| 194 | 0xC2 | float32 | Current set-point |
| 195 | 0xC3 | 3×float32 | Output voltage, current, power (12 bytes) |
| 196 | 0xC4 | float32 | Internal temperature (°C) |
| 197-208 | 0xC5-0xD0 | float32 | Preset M1-M6 voltage/current (see §5.3) |
| 214 | 0xD6 | uint8 | Display brightness |
| 215 | 0xD7 | uint8 | Volume (beep level) |
| 217 | 0xD9 | float32 | Ah counter (amp-hours) |
| 218 | 0xDA | float32 | Wh counter (watt-hours) |
| 219 | 0xDB | uint8 | Output state (0=off, 1=on) |
| 220 | 0xDC | uint8 | Protection status (see §6) |
| 221 | 0xDD | uint8 | Regulation mode (0=CC, 1=CV) |
| 222 | 0xDE | string | Model name |
| 223 | 0xDF | string | Hardware version |
| 224 | 0xE0 | string | Firmware version |
| 225 | 0xE1 | uint8 | Device address (1-255, for multi-device verification) |
| 226 | 0xE2 | float32 | Max voltage capability |
| 227 | 0xE3 | float32 | Max current capability |
| 255 | 0xFF | block | Full state dump (see §7) |

### 5.2 Writable Registers (c2 = 0xB1)

| Register | Hex | Type | Description |
|----------|-----|------|-------------|
| 193 | 0xC1 | float32 | Set voltage |
| 194 | 0xC2 | float32 | Set current limit |
| 197 | 0xC5 | float32 | Preset M1 voltage |
| 198 | 0xC6 | float32 | Preset M1 current |
| 199 | 0xC7 | float32 | Preset M2 voltage |
| 200 | 0xC8 | float32 | Preset M2 current |
| 201 | 0xC9 | float32 | Preset M3 voltage |
| 202 | 0xCA | float32 | Preset M3 current |
| 203 | 0xCB | float32 | Preset M4 voltage |
| 204 | 0xCC | float32 | Preset M4 current |
| 205 | 0xCD | float32 | Preset M5 voltage |
| 206 | 0xCE | float32 | Preset M5 current |
| 207 | 0xCF | float32 | Preset M6 voltage |
| 208 | 0xD0 | float32 | Preset M6 current |
| 209 | 0xD1 | float32 | OVP threshold (volts) |
| 210 | 0xD2 | float32 | OCP threshold (amps) |
| 211 | 0xD3 | float32 | OPP threshold (watts) |
| 212 | 0xD4 | float32 | OTP threshold (°C) |
| 213 | 0xD5 | float32 | LVP threshold (volts) |
| 214 | 0xD6 | uint8 | Display brightness |
| 215 | 0xD7 | uint8 | Volume (beep level) |
| 216 | 0xD8 | uint8 | Metering enable (0=stop, 1=start) |
| 219 | 0xDB | uint8 | Output enable (0=off, 1=on) |

> **Note on brightness range:** The official app writes 0-255, but the Python library
> enforces 0-10. The actual hardware range needs verification on-device.

### 5.3 Preset Register Formula

For preset M*n* (n = 1..6):
- **Voltage register** = `0xC3 + 2*n` = `0xC5, 0xC7, 0xC9, 0xCB, 0xCD, 0xCF`
- **Current register** = `0xC3 + 2*n + 1` = `0xC6, 0xC8, 0xCA, 0xCC, 0xCE, 0xD0`

---

## 6. Protection Status Codes (Register 0xDC)

| Value | Status | Description |
|-------|--------|-------------|
| 0 | OK | Normal operation |
| 1 | OVP | Over-voltage protection triggered |
| 2 | OCP | Over-current protection triggered |
| 3 | OPP | Over-power protection triggered |
| 4 | OTP | Over-temperature protection triggered |
| 5 | LVP | Low/under-voltage protection triggered |
| 6 | REP | Reverse polarity/connection protection triggered |

---

## 7. Full State Dump (Register 0xFF)

Request: `F1 A1 FF 01 00 00`

Response payload is ~139 bytes. All float values are IEEE 754 single-precision, little-endian.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0x00 (0) | 4 | d1 | Input voltage (V) |
| 0x04 (4) | 4 | d2 | Voltage set-point (V) |
| 0x08 (8) | 4 | d3 | Current set-point (A) |
| 0x0C (12) | 4 | d4 | Measured output voltage (V) |
| 0x10 (16) | 4 | d5 | Measured output current (A) |
| 0x14 (20) | 4 | d6 | Measured output power (W) |
| 0x18 (24) | 4 | d7 | Temperature (°C) |
| 0x1C (28) | 4 | d8 | Preset M1 voltage |
| 0x20 (32) | 4 | d9 | Preset M1 current |
| 0x24 (36) | 4 | d10 | Preset M2 voltage |
| 0x28 (40) | 4 | d11 | Preset M2 current |
| 0x2C (44) | 4 | d12 | Preset M3 voltage |
| 0x30 (48) | 4 | d13 | Preset M3 current |
| 0x34 (52) | 4 | d14 | Preset M4 voltage |
| 0x38 (56) | 4 | d15 | Preset M4 current |
| 0x3C (60) | 4 | d16 | Preset M5 voltage |
| 0x40 (64) | 4 | d17 | Preset M5 current |
| 0x44 (68) | 4 | d18 | Preset M6 voltage |
| 0x48 (72) | 4 | d19 | Preset M6 current |
| 0x4C (76) | 4 | d20 | OVP set-point (V) |
| 0x50 (80) | 4 | d21 | OCP set-point (A) |
| 0x54 (84) | 4 | d22 | OPP set-point (W) |
| 0x58 (88) | 4 | d23 | OTP set-point (°C) |
| 0x5C (92) | 4 | d24 | LVP set-point (V) |
| 0x60 (96) | 1 | d25 | Brightness (0-255) |
| 0x61 (97) | 1 | d26 | Volume (beep level) |
| 0x62 (98) | 1 | d27 | Metering state (0=open/running, 1=closed/stopped) |
| 0x63 (99) | 4 | d28 | Ah counter (amp-hours) |
| 0x67 (103) | 4 | d29 | Wh counter (watt-hours) |
| 0x6B (107) | 1 | d30 | Output state (0=off, 1=on) |
| 0x6C (108) | 1 | d31 | Protection status (0-6, see §6) |
| 0x6D (109) | 1 | d32 | CV/CC mode (0=CC, 1=CV) |
| 0x6E (110) | 1 | d33 | Unknown (reads as a byte, purpose unclear) |
| 0x6F (111) | 4 | d34 | Max voltage capability (V) |
| 0x73 (115) | 4 | d35 | Max current capability (A) |
| 0x77 (119) | 4 | d36 | OVP max ceiling (V) |
| 0x7B (123) | 4 | d37 | OCP max ceiling (A) |
| 0x7F (127) | 4 | d38 | OPP max ceiling (W) |
| 0x83 (131) | 4 | d39 | OTP max ceiling (°C) |
| 0x87 (135) | 4 | d40 | LVP max ceiling (V) |

> **d36-d40 (offsets 0x77-0x87):** These are the maximum configurable ceilings for each
> protection threshold. cho45 reads them but marks them as unknown. The .NET decompilation
> confirms they correspond to OVP/OCP/OPP/OTP/LVP maximums, in that order.

**Default capabilities** (before device reports actual values):
- Max voltage: 24.0V
- Max current: 5.0A

---

## 8. Data Types

All numeric values transmitted over the wire use:

| Type | Size | Encoding |
|------|------|----------|
| float32 | 4 bytes | IEEE 754 single-precision, **little-endian** |
| uint8 | 1 byte | Unsigned byte |
| string | variable | ASCII (null-terminated or length-delimited by packet) |

---

## 9. Concrete Packet Examples

### Session Open (Connect)
```
TX: F1 C1 00 01 01 02
     │  │  │  │  │  └─ checksum: (0x00+0x01+0x01) & 0xFF = 0x02
     │  │  │  │  └──── data: 0x01 (enable)
     │  │  │  └─────── length: 1
     │  │  └────────── register: 0x00
     │  └───────────── command: 0xC1 (session control)
     └──────────────── header: 0xF1 (host→device)
```

### Session Close (Disconnect)
```
TX: F1 C1 00 01 00 01
```

### Read All Registers
```
TX: F1 A1 FF 01 00 00
```

### Output ON (RUN)
```
TX: F1 B1 DB 01 01 DD
```

### Output OFF (STOP)
```
TX: F1 B1 DB 01 00 DC
```

### Set Voltage to 5.0V
```
TX: F1 B1 C1 04 00 00 A0 40 A5
                 └──────────┘
                 IEEE 754 LE: 5.0
     checksum: (0xC1+0x04+0x00+0x00+0xA0+0x40) & 0xFF = 0xA5
```

### Set Current to 1.0A
```
TX: F1 B1 C2 04 00 00 80 3F 85
                 └──────────┘
                 IEEE 754 LE: 1.0
```

### Set Baud Rate to 115200
```
TX: F1 B0 00 01 05 06
                 └── baud index 5 = 115200
```

### Set Volume to 9
```
TX: F1 B1 D7 01 09 E1
```

### Set Brightness to 5
```
TX: F1 B1 D6 01 05 DC
```

### Set OVP to 25.0V
```
TX: F1 B1 D1 04 00 00 C8 41 4E
```

### Enable Metering
```
TX: F1 B1 D8 01 01 DA
```

### Read Model Name
```
TX: F1 A1 DE 01 00 DF
RX: F0 A1 DE XX [string bytes...] [checksum]
```

### Read Firmware Version
```
TX: F1 A1 E0 01 00 E1
RX: F0 A1 E0 XX [string bytes...] [checksum]
```

### Enter Firmware Upgrade (DFU) Mode
```
TX: F1 C0 00 01 01 02
```

> **WARNING:** This does NOT perform a normal restart. The device enters its bootloader
> (firmware upgrade mode) and the USB CDC port disappears. A physical USB unplug/replug
> is required to return to normal operation. No reference implementation uses this command.
> It was identified from the .NET decompilation's `CMD_192` handler.

---

## 10. Response Parsing

Responses from the device use header byte `0xF0` instead of `0xF1`. The rest of the structure is identical:

```
F0 [command] [register] [length] [data...] [checksum]
```

To parse a response buffer that may contain multiple packets:
1. Find packet boundary: total size = `5 + buffer[offset + 3]`
2. Minimum valid response is 5 bytes (header + command + register + length + checksum)
3. Extract data payload: bytes from offset+4 to offset+4+length (exclusive of checksum)
4. Dispatch based on register byte (offset+2)

---

## 11. PC-Side Automation Features

These features are implemented entirely in the PC software, not in device firmware. They work by sending repeated set-voltage/set-current commands with timing delays.

### Voltage Scan (sweep current at fixed voltage)
| Parameter | Type | Description |
|-----------|------|-------------|
| V | float | Fixed voltage |
| startA | float | Starting current |
| stopA | float | Ending current |
| stepA | float | Current increment per step |
| sleepTime | float | Delay between steps (seconds) |

### Current Scan (sweep voltage at fixed current)
| Parameter | Type | Description |
|-----------|------|-------------|
| A | float | Fixed current |
| startV | float | Starting voltage |
| stopV | float | Ending voltage |
| stepV | float | Voltage increment per step |
| sleepTime | float | Delay between steps (seconds) |

### Table/Sequence Mode
| Parameter | Type | Description |
|-----------|------|-------------|
| sn | int | Step sequence number |
| V | float | Voltage for this step |
| A | float | Current for this step |
| sleepTime | float | Dwell time (seconds) |

Up to 10 steps, loopable N times with configurable start/stop rows.

---

## 12. Notes and Errata

- The device defaults to CV mode (d32=1) on initialization.
- After writing preset values, send a full read-all (0xFF) to refresh state — the device does not echo individual register writes.
- Temperature is reported in Celsius by the device.
- Offset 0x6E (byte 110) in the full dump is not read by the official app. The gap between offset 109 (CV/CC mode) and 111 (max voltage) is simply skipped. Treat as reserved.
- Protection naming: register 0xD5 and protection code 5 are referred to as both "UVP" (under-voltage) and "LVP" (low-voltage) across implementations. They are the same thing.
- Protection code 6 (REP): documented by cho45, Python, and Rust implementations but **not handled** by the official Windows app. The device likely sends it for reverse polarity events, but the official app ignores it.
- Read requests: the official app sends reads with length=1 and a zero data byte (e.g. `F1 A1 DE 01 00 DF`). Some third-party implementations send reads with length=0 and no data byte (e.g. `F1 A1 DE 00 DE`). Both formats are accepted by the device.
- **Checksum validation may be lenient:** The official app has a bug in `CmdMoudle.setContent()` where modifying a command's payload does not recalculate the checksum. This means the baud rate command (CMD_13) is sent with an incorrect checksum, yet the device accepts it. Implementations should still compute correct checksums, but be aware the device may not strictly validate them on the RX side.
- Volume (offset 0x61 / register 0xD7): present in the full state dump and writable as a register, but the official Windows app never exposes it in the UI. Third-party implementations (cho45, Python, Rust) all document and use it.
- **Command 0xC0 is NOT a restart — it enters firmware upgrade (DFU/bootloader) mode.** The USB CDC port disappears and a physical unplug/replug is required to recover. None of the 4 reference implementations use this command. It was originally labeled "System Restart" based on the .NET decompilation, but on-device testing confirms it enters the bootloader. There is no known command for a normal soft restart.

---

## 13. Cross-Reference Confidence

| Feature | Official App | cho45 (JS) | svenk123 (C) | KochC (Python) | Rust |
|---------|:---:|:---:|:---:|:---:|:---:|
| Packet structure | Y | Y | Y | Y | Y |
| Checksum algorithm | Y | Y | Y | Y | Y |
| Session connect/disconnect | Y | Y | Y | Y | partial |
| Voltage/current set (float32) | Y | Y | N (byte only) | Y | N (byte only) |
| Preset registers (M1-M6) | Y | Y | - | Y | Y |
| Protection thresholds (float32) | Y | Y | N (byte only) | Y | Y |
| Volume register (0xD7) | Y* | Y | - | Y | Y |
| Metering enable (0xD8) | Y | Y | - | Y | - |
| Full state dump (0xFF) | Y | Y | - | Y | partial |
| Protection ceiling fields (d36-d40) | Y | - | - | - | - |
| REP protection (code 6) | - | Y | - | Y | Y |
| Auto-pushed telemetry | Y | Y | - | Y | - |
| USB VID:PID | - | Y | - | - | - |

`Y` = implemented/documented, `-` = not covered, `N` = different approach, `*` = byte exists in dump but not exposed in UI
