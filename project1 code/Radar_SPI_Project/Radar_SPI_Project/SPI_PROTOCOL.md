# RadarWire v1 — UNO 3 ↔ UNO 1 SPI Interface Specification

**Status:** implemented as a reference protocol in the source files supplied in this package; **not physically verified** on the combined two-board hardware. This document is the authoritative, byte-exact specification for these sources; the original outline calls the 48-byte structure a *proposal*. Wire version is `0x01` and changes to these layouts require a new protocol version.

## 1. Roles, scope, design rationale

| Endpoint | AVR role | Electrical role | Source | Data handled |
|---|---|---|---|---|
| UNO 3 scanner | **SPI master** | Produces SCK and SS, transmits MOSI, receives MISO | `UNO3_SPI_Scanner/UNO3_SPI_Scanner.ino` | Local stepper, Hall, ToF, clustering, object tracking, telemetry |
| UNO 1 central hub | **SPI slave** | Responds on MISO **only when UNO 3 clocks** | `UNO1_SPI_Hub_Reference/UNO1_SPI_Hub_Reference.ino` | Receives scanner snapshots, queues mode + selected target commands |
| UNO 2 interface | UART endpoint to UNO 1 | Not an SPI participant | application-specific / not delivered as full subsystem | May request modes via UNO 1 UART |
| UNO 4 turret | I²C endpoint to UNO 1 | Not an SPI participant | application-specific / not delivered | Out of scope for this scanner implementation |
| PC monitor | USB UART receiver of UNO 3 only | **No commands transmitted** | `uno3_spi_radar_gui.py` | Reads status + BOTH coordinate systems |

The UNO 3 master clocks a **48-byte full-duplex** exchange nominally every **100 ms**. During one exchange, each MOSI byte from UNO 3 transfers concurrently with one MISO byte from UNO 1. The MISO message is **pre-staged** by UNO 1, not calculated in real time from the MOSI data currently arriving. UNO 1 cannot originate its own SPI transaction in this wiring. Thus, commands are observed by UNO 3 at the *end* of its next polling transaction, and UNO 1 observes the resulting telemetry on subsequent transactions. A received command cannot retroactively affect the snapshot transferred alongside it.

**Output is processed tracks, never all raw ToF samples.** There are at most 3 object slots, in accordance with the original outline.

## 2. Electrical interface and wire-level timing

| Wire | UNO 3 | UNO 1 | Direction |
|---|---|---|---|
| SS / CS | D9 output | D10 / PB2 / SS input | UNO 3 → UNO 1 (LOW = selected) |
| MOSI | D11 | D11 | UNO 3 → UNO 1 |
| MISO | D12 | D12 | UNO 1 → UNO 3 |
| SCK | D13 | D13 | UNO 3 → UNO 1 |
| Signal reference | GND | GND | **Common ground mandatory** |
| UNO 3 physical SS | D10 **OUTPUT / HIGH** | **not connected** | Required to keep ATmega328P in master mode |

No separate MOSI/MISO crossing is needed: connect like-named SPI pins. Each board operates at compatible 5 V Uno logic levels. Keep wires short, avoid noisy stepper power wiring, do not hot-plug, and do not expose SPI to motors' supply voltage. No SPI pull-up or level shifter is specified for two directly connected 5 V Uno R3-class boards; for different-voltage devices use proper level conversion. UNO 3's I²C ToF uses A4/A5 and motor pins D2/D3/D4, Hall D7/A0 independently. UNO 1's D0/D1 can run UART to UNO 2 and A4/A5 I²C to UNO 4; UNO 1 must schedule those activities so SPI interrupts can run in time.

| Parameter | Value in reference | Meaning |
|---|---|---|
| SPI mode | **0** | CPOL=0, CPHA=0: sample on rising SCK edge, change data on falling edge |
| Bit order | **MSB first** | Byte bits 7 down through 0 |
| SPI clock | **125,000 Hz** | SPI clock when transfer active (nominal 16 MHz / 128 AVR divider) |
| Bit time | **8 μs** | At 125 kHz |
| Clocked byte duration | **64 μs** | Eight clock periods, excluding software overhead |
| Minimum deliberate inter-byte gap | **100 μs** | `delayMicroseconds(100)` AFTER **each** `SPI.transfer()` |
| SS setup | **120 μs** | Before first `SPI.transfer`, after CS LOW |
| Transaction length | **48 bytes in each direction** | Exactly 48 simultaneous byte transfers under one continuous CS LOW |
| CS idle | **HIGH** | D9 goes HIGH after 48th byte and stays HIGH between transfers |
| Nominal start-to-start cadence | **100 ms** | Cooperative loop may introduce jitter |
| Approx. deliberate+wire transaction duration | **7.992 ms**, plus code overhead | `48 × (64+100) μs + 120 μs`; not an oscilloscope measurement |
| Command link timeout | **3000 ms** | From last successfully CRC-validated incoming command; only applies after first valid frame |

UNO 3 additionally services its motor-position history and Hall edge observation between some SPI bytes, to avoid losing bearing history during the ~8 ms transfer. These cooperative callbacks execute with interrupts enabled.

The UNO 1 slave uses the AVR SPI-transfer-complete ISR (`SPI_STC_vect`) to save one incoming byte and preload the following outgoing byte. The D10 pin-change ISR (`PCINT0_vect`, PB2 only) detects CS LOW to reset the byte index and prime `SPDR` with byte 0; CS HIGH finalizes exactly 48 bytes, or marks an incomplete frame. Dedicated double-buffered command data prevents the main loop from editing the packet being transmitted. The intentional 100 μs gap is a conservative timing accommodation for interrupt latency while the hub also uses UART/I²C. It is **not a guarantee** under arbitrary additional interrupt load. Measure CS/SCK/MISO with a logic analyzer and verify ISR latency on the final UNO 1 sketch, especially when integrating UNO 4 I²C work.

### Transfer timeline

```
Time →
UNO3 D9 (CS)    ─────────┐_____________________________________┌────────
                         │ 120 μs setup                         │
UNO3 D13 (SCK)  ___________||||||||___gap___||||||||___..._____||||||||___
UNO3 D11 (MOSI)            TX[0]          TX[1]             TX[47]
UNO1 D12 (MISO)            RX[0]          RX[1]             RX[47]
                         ↑ first command byte was preloaded by UNO1
UNO3 afterwards: validate command; optionally dispatch new command sequence
UNO1 afterwards:  validate incoming telemetry; publish parsed hub snapshot
```

A new SPI master implementation **must** honor the full 48-byte CS-low span and the interbyte delay. Do not lower CS between bytes; a CS toggle marks a new frame. If the SPI device produces an unexpected number of bytes, discard its partial frame. If reimplementing UNO 1 on other hardware, queue the first byte *before* the first SCK edge.

## 3. Universal frame rules

- Frames are **48 octets**, indexed `0` through `47`; no delimiter, padding outside the frame, or escaping.
- Byte **0**: sync `0xA5`; byte **1**: protocol version `0x01`; byte **2**: per-direction sequence counter (unsigned 8-bit, wraps from 255 to 0).
- **47**: CRC-8/ATM over **bytes 0–46 inclusive**, including header, payload, reserved bytes; sender MUST set byte 47 to CRC.
- CRC-8/ATM: **polynomial `0x07`, init `0x00`, reflected input `false`, reflected output `false`, XOR-out `0x00`, no augmented zero bytes**. Check vector ASCII `123456789` has CRC `0xF4`.
- All 16-bit values are **little-endian** (`low byte` first), even though byte transfer bit order is MSB-first. Signed values use 16-bit two's complement. Byte quantities are unsigned unless specified.
- Reserved bytes in **UNO 1 commands MUST be zero**. Reserved telemetry object slots and bytes are zero, unless a later version defines them.
- CRC-valid but unknown-version and wrong sync frames are rejected. Version extension must not change v1 behavior silently.
- CRC detects accidental transmission corruption, not security attacks; this SPI bus is an intra-device, physically trusted connection.

### CRC reference code (portable C/C++)

```c
#include <stdint.h>
uint8_t radar_crc8(const uint8_t *p, uint8_t n) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}
/* after filling bytes 0–46: frame[47] = radar_crc8(frame, 47); */
```

## 4. UNO 3 → UNO 1: MOSI telemetry frame

| Offset | Width | Name | Type | Detailed meaning |
|---:|---:|---|---|---|
| 0 | 1 | `sync` | u8 | `0xA5` |
| 1 | 1 | `version` | u8 | `0x01` |
| 2 | 1 | `telemetry_seq` | u8 | Incremented for **every initiated SPI frame**, modulo 256. Detect gaps or duplicates at UNO 1. |
| 3 | 1 | `active_mode` | u8 | `0 STOP`, `1 SEARCH_SLOW`, `2 SEARCH_FAST`, `3 TRACK_TARGET`. Calibration is in **state**, not a separate active mode. |
| 4 | 1 | `status_flags` | u8 | Bitfield below; includes coordinate format |
| 5 | 1 | `object_count` | u8 | `0..3`; object slots beginning at 6 populated consecutively |
| 6–17 | 12 | `object[0]` | bytes | Slot 0 in selected layout, zero when unoccupied |
| 18–29 | 12 | `object[1]` | bytes | Slot 1 in selected layout, zero when unoccupied |
| 30–41 | 12 | `object[2]` | bytes | Slot 2 in selected layout, zero when unoccupied |
| 42 | 1 | `scanner_state` | u8 | `0 CALIBRATION`, `1 IDLE`, `2 MOVING`, `3 FAULT` |
| 43 | 1 | `hall_strikes` | u8 | Consecutive Hall-reference misses (saturates/cleared on valid check) |
| 44 | 1 | `selected_target_id` | u8 | Selected ID currently in effect; `0` = tracker may round-robin |
| 45 | 1 | `accepted_cmd_seq` | u8 | Most recently **accepted and dispatched** valid command sequence; initial `0xFF` before first command, wrap caveat below |
| 46 | 1 | `last_error` | u8 | Error enum table below; errors may remain until accepted subsequent command |
| 47 | 1 | `crc` | u8 | CRC-8/ATM bytes 0–46 |

### Status flags (byte 4)

| Bit / mask | Name | One means | Zero means |
|---|---|---|---|
| 0 `0x01` | `HOMED` | Hall home is calibrated and valid | Uncalibrated / invalidated |
| 1 `0x02` | `TOF_READY` | Sensor initialized, not flagged stale | Offline, initialization failed, or stale |
| 2 `0x04` | `TORQUE_HOLD` | Firmware commanded ENA **HOLD** | Firmware commanded **FREE** |
| 3 `0x08` | `HALL_ACTIVE` | Hall digital threshold active | Hall digital inactive |
| 4 `0x10` | `REHOME_PENDING` | Return-to-calibration marked by Hall misses | No pending drift-induced recalibration |
| 5 `0x20` | `CYLINDRICAL` | All 3 object slots use **cylindrical** encoding | All use **Cartesian** encoding |
| 6 `0x40` | `TARGET_LOST` | A track/selected target loss has been reported since last cleared by acquisition/mode change | Not currently flagged |
| 7 `0x80` | `LINK_TIMEOUT` | No valid command frames for over 3000 ms after link was established | Currently receiving valid frames or not yet established |

**ERROR enum in byte 46:** `0 NONE`, `1 RX_CRC`, `2 RX_HEADER`, `3 BAD_COMMAND`, `4 NOT_HOMED`, `5 TOF_UNAVAILABLE`, `6 LINK_TIMEOUT`, `7 TOF_STALE`, `8 HALL_DRIFT`, `9 TRACK_LOST`; `10..255` reserved.

## 5. Twelve-byte object records — Cartesian and cylindrical

Object slots are interpreted according to **byte 4 bit 5**, never by heuristic. Both layouts have the same size, prefix, suffix and object count. There is **no 24-byte combined record** inside SPI: selecting one wire format leaves room for three objects. UNO 3 internally stores Cartesian targets, computes cylindrical output as needed, and its USB telemetry separately presents both at once.

### Shared object prefix/suffix

| Relative offset | Width | Type | Field | Rule |
|---:|---:|---|---|---|
| 0 | 1 | u8 | `id` | `1..255` for occupied slots; `0` for empty slots |
| 1 | 1 | u8 | `object_flags` | See bitfield below |
| 10 | 1 | u8 | `quality` | `0..255` heuristic, currently 60/150/220 for 1/2/3+ clustered points; not an accuracy guarantee |
| 11 | 1 | u8 | `age_scans` | Count of relevant scan legs missed, subject to scanner tracking policy |

Object flags at relative offset 1: `0x01 velocity_valid`, `0x02 in_bounds`, `0x04 stale_age_nonzero`; bits 3–7 MUST be treated as reserved. Velocities are **zero when invalid**; consumers MUST use the flag, not value zero, to decide validity. `age_scans>0` means last known location is stale, not that a new ToF measurement occurred in that frame. A slot may retain an object for several missed observations. On expiration, it is omitted from the next count/snapshot; IDs may eventually wrap or be reused.

### Cartesian slot (bit `CYLINDRICAL` clear)

| Relative offset | Width | Type | Field | Units |
|---:|---:|---|---|---|
| 2–3 | 2 | i16 LE | `x_mm` | mm relative to lower-left world origin |
| 4–5 | 2 | i16 LE | `y_mm` | mm relative to lower-left world origin |
| 6–7 | 2 | i16 LE | `vx_mm_s` | mm/s, world +X direction |
| 8–9 | 2 | i16 LE | `vy_mm_s` | mm/s, world +Y direction |

### Cylindrical/polar slot (bit `CYLINDRICAL` set)

| Relative offset | Width | Type | Field | Units |
|---:|---:|---|---|---|
| 2–3 | 2 | u16 LE | `r_mm` | mm from **scanner axis at (500,500)**, not sensor housing wall |
| 4–5 | 2 | u16 LE | `theta_cdeg` | **centidegrees**, `0..35999` (e.g. 9000 = 90.00°) |
| 6–7 | 2 | i16 LE | `vr_mm_s` | radial mm/s, positive increasing range |
| 8–9 | 2 | i16 LE | `vt_mm_s` | **tangential linear velocity** in mm/s, positive CCW, **NOT angular degrees/s** |

The original outline called this last field `vtheta_mm_s` while describing tangential speed. This implementation names it `vt_mm_s` to avoid conflating linear and angular units. This is a **clarifying protocol decision** and does not imply empirical motion-calibration improvements.

### World geometry and conversions

- The area is a nominal **1000 × 1000 mm square**, origin `(0,0)` at lower left; positive X right and positive Y up when seen from above.
- UNO 3 rotation axis assumed at `(500,500)` mm. Bearing zero is along world **+X**, increasing **counterclockwise**.
- `theta_rad = (theta_cdeg / 100) × π/180` (value normalized to `[0,2π)`).
- Convert cylindrical → Cartesian: `x = 500 + r cos(theta_rad)` and `y = 500 + r sin(theta_rad)`.
- Convert Cartesian → cylindrical: `dx=x−500`, `dy=y−500`; `r=sqrt(dx²+dy²)`; `theta=atan2(dy,dx)` normalized to 0–360°, then round to centidegrees.
- Convert Cartesian velocity → cylindrical: `vr=(dx·vx+dy·vy)/r`, `vt=(−dy·vx+dx·vy)/r` for `r>0`; if `r=0`, set both zero.
- Convert cylindrical velocity → Cartesian: `vx=vr·cos(theta)−vt·sin(theta)` and `vy=vr·sin(theta)+vt·cos(theta)`.
- `omega_rad_s = vt_mm_s / r_mm` only if `r>0`. This is **derived**, not directly transported.
- Rounding and fixed-point quantization mean the two encodings are not exactly identical after conversion. An off-axis/radial sensor mount must be calibrated in UNO 3, not corrected by redefining the wire coordinate origin.

Examples: `x=800, y=500` corresponds to `r=300 mm, theta=0.00°`; `x=500, y=800` corresponds to `r=300 mm, theta=90.00°`. A cylindrical `vr=15, vt=-20` at 90° corresponds approximately to Cartesian `vx=20, vy=15 mm/s`.

### Source configuration switch

In `UNO3_SPI_Scanner.ino`, edit **exactly one** compile-time line and upload UNO 3 again:

```cpp
const uint8_t SPI_COORDINATE_MODE = RadarWire::COORD_CARTESIAN;
// or, instead:
// const uint8_t SPI_COORDINATE_MODE = RadarWire::COORD_CYLINDRICAL;
```

Keep **only one** declaration active. UNO 1 does not require reflashing to change this setting: it reads `status_flags & 0x20` on each frame and decodes accordingly. The PC monitor always receives and displays **both** coordinate representations in its `OBJECT` lines, independent of the SPI format switch.

## 6. UNO 1 → UNO 3: MISO command frame

| Offset | Width | Name | Type | Required contents |
|---:|---:|---|---|---|
| 0 | 1 | `sync` | u8 | `0xA5` |
| 1 | 1 | `version` | u8 | `0x01` |
| 2 | 1 | `command_seq` | u8 | Increment for each **new** operator/hub command; wrap permitted |
| 3 | 1 | `requested_mode` | u8 | `0 STOP`, `1 SEARCH_SLOW`, `2 SEARCH_FAST`, `3 TRACK_TARGET`, `4 REHOME` |
| 4 | 1 | `command_flags` | u8 | Only bit 0 defined: `0x01 TRY_REINIT_TOF` when **not moving**; other bits zero |
| 5 | 1 | `selected_target_id` | u8 | `0` all/round-robin; `1..255` specific ID for TRACK_TARGET |
| 6–46 | 41 | `reserved` | bytes | **All zero**, otherwise UNO 3 rejects the command |
| 47 | 1 | `crc` | u8 | CRC-8/ATM over bytes 0–46 |

`REHOME=4` directly enters manual Hall calibration; **this is a request to stop pulses and release torque**, not a command to power-driven-home. After five seconds of steady Hall active input, UNO 3 captures its current step as zero and automatically **re-engages holding torque**. It resumes the previous active mode if one existed when recalibration began; otherwise remains in STOP/IDLE until a new SPI command. A STOP received during calibration does not cancel the safe manual calibration process.

`selected_target_id=0` with TRACK_TARGET means round-robin candidate selection, preserving the standalone prototype behavior. Specific nonzero ID requests are not guaranteed to exist; on missing object the scanner may return to slow search and flag target loss. For modes other than TRACK_TARGET, UNO 3 stores the supplied selected ID but does not necessarily use it for current motion.

**Never send torque FREE/HOLD commands over SPI.** Torque policy is local firmware safety behavior: only CALIBRATION commands FREE; all other states command HOLD. The SPI master never reads PC input as commands; all control flows UNO 2 → UNO 1 → SPI → UNO 3.

### Sequencing, acknowledgments, stale frames

1. UNO 1 stages a complete MISO command frame ahead of the next SPI exchange; initial command is `seq=0`, `STOP`, target 0.
2. UNO 3 reads all 48 MISO bytes as it transmits its own telemetry.
3. UNO 3 checks sync, version, CRC, mode `0..4`, allowed flags, and zero reserved bytes. Invalid frames do not change the requested mode or accepted sequence; protocol error is reported in subsequent telemetry.
4. For valid bytes, UNO 3 refreshes command-link liveness. If `command_seq` equals the last accepted sequence, it treats the frame as a **duplicate** and does not repeat calibration or mode side effects. Otherwise it records the new sequence, checks local scanner eligibility and processes its command.
5. UNO 3 includes `accepted_cmd_seq` in **future** telemetry at byte 45; it cannot acknowledge a MISO frame within the MOSI packet that was prepared before receiving it. **At least one additional polling exchange** is required for confirmation.
6. UNO 1 should keep the same command bytes and sequence until the operator/hub issues a **new** command. Increment `command_seq` modulo 256 for every new command, including one that repeats a mode to trigger a retry.
7. Command ACK indicates successfully parsed/accepted sequence, **not** that physical movement finished or that the requested mode was viable. Check `scanner_state`, `active_mode`, `last_error`, `HOMED`, `TOF_READY` and `TORQUE_HOLD` for execution.
8. `0xFF` in ACK byte 45 means *no accepted command yet* immediately at startup, but after sequence rollover `0xFF` is a legitimate last accepted command value. Resolve startup ambiguity through the expected `seq=0` handshake and ongoing telemetry chronology.
9. Each MOSI telemetry sequence increments for every initiated transaction; 8-bit counter rollover is normal. A gap greater than 1 modulo 256 indicates at least one missed snapshot **if fewer than 256 snapshots elapsed**. CRC/byte count also detect transport faults.
10. There is no cryptographic authentication, transport retransmission, command execution queue, or guaranteed delivery. The hub owns retry policy, but repeated *identical sequence numbers* are intentionally idempotent.

**3-second loss policy:** UNO 3 starts its timeout clock at its first CRC-valid MISO frame. If no CRC-valid command is received for over 3000 ms, it sets the timeout flag and error. If scanning, it stops pulses, commands **HOLD**, invalidates home because it stopped mid-sweep, clears objects, and requires a fresh calibration before any subsequent scan. If link loss occurs **during calibration**, a pending automatic scan resume is cancelled: the scanner may still finish manual Hall alignment and engage HOLD, but will not resume motor motion without a healthy link and a new command. A valid later duplicate command restores liveness but **does not restart a stopped scan**; send a new sequence and, if unhomed, REHOME first. This is a chosen fail-safe policy, not something specified quantitatively in the original outline.

### Mode and state distinctions

| Hub mode | Active scanner mode | Scanner state | Result |
|---|---|---|---|
| 0 STOP | 0 | IDLE, or still CALIBRATION | No motor STEP pulses; HOLD if outside calibration |
| 1 SEARCH_SLOW | 1 | MOVING if homed and ToF ready | Full out-and-back scanning nominal 0.5 rev/s peak |
| 2 SEARCH_FAST | 2 | MOVING if homed and ToF ready | Full out-and-back scanning nominal 1 rev/s peak |
| 3 TRACK_TARGET | 3, potentially SEARCH_SLOW fallback | MOVING when ready | Narrow ±30° tracking, 4 visits, full awareness refresh |
| 4 REHOME | 0 until complete | CALIBRATION → IDLE | Motor FREE while manually aligned, HOLD after five seconds; may auto-resume prior mode |

Real RPM, sample latency, tracking accuracy, Hall behavior and hardware torque polarity must be validated. The current motor controller assumes 3200 pulses per revolution, TB6600 1/16 microstep setting, and active-low common-anode STEP/DIR. The Hall input is active LOW, and the five-second countdown resets if the magnet leaves the active zone. Three unsuccessful full-forward Hall return checks request recalibration after the safe return leg.

## 7. Deterministic example frames

The examples below are exact v1 packets, spaces separating hex octets. Any omitted physical framing would be invalid. You can regenerate with `radar_spi_protocol.py`.

**Initial STOP command**, `seq 0`, target 0, CRC `0x23`:

```text
a5 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 23
```

**TRACK_TARGET command** `seq 1`, target 2, CRC `0xA1`:

```text
a5 01 01 03 00 02 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a1
```

**Cartesian telemetry**, `telemetry_seq=22 (0x16)`, `mode=SEARCH_FAST`, flags `0x07` (homed + ToF + hold), `object_count=1`, object ID 7 at `(800,500) mm`, velocities `(-10,+5) mm/s`, quality 210, age 0; state MOVING, Hall strikes 0, target 2, ACK seq 21 (`0x15`), error 0, CRC `0x17`:

```text
a5 01 16 02 07 01 07 03 20 03 f4 01 f6 ff 05 00
 d2 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 02 00 02 15 00 17
```

Here object slot 0 begins at offset 6 (`07 03` = ID 7, valid velocity + in bounds); `20 03` is x 800; `f4 01` is y 500; `f6 ff` is signed vx −10; `05 00` is vy 5. Only occupied slots count; unused slots are all zero. `flags=0x07` refers to the **frame status**, not object flags `0x03`.

## 8. UNO 1 UART-facing reference application

The included UNO 1 sketch **only illustrates** SPI and its UNO 2 UART interaction. It is **not** the final UNO 1 application and does not initialize/control the UNO 4 turret. UART is hardware D0/D1 at 115200 8N1. Its ASCII protocol is distinct from binary RadarWire SPI:

| Direction | Line format | Purpose |
|---|---|---|
| UNO 2 → UNO 1 | `CMD|mode|target\n` | Mode in `0..4`, target `0..255`; increments SPI sequence; e.g. `CMD|2|0` |
| UNO 2 → UNO 1 | `RETRY\n` | Sets ToF re-init flag and increments sequence; carries last desired mode |
| UNO 1 → UNO 2 | `RAD|seq|mode|state|statusFlags|count|strikes|selected|ack|error|coordFormat` | One summary per valid SPI telemetry frame |
| UNO 1 → UNO 2 | `OBJ|id|x|y|r|thetaCdeg|vx|vy|vr|vt|quality|age|flags` | One decoded target record per occupied slot, with **both coordinate sets**, synthesized from the selected SPI wire representation |
| UNO 1 → UNO 2 | `HELLO|UNO1|SPI_SLAVE_REFERENCE|1` | Boot identifier |

Potential UART traffic can block main-loop parsing even if interrupts remain enabled. In a final UNO 1 integrating UNO 4 I²C and a GUI protocol, use a nonblocking queued UART output policy and verify no SPI byte losses. Any new I²C ISR or long interrupt-disabled region must be assessed against the 100 μs inter-byte delay.

## 9. UNO 3 USB telemetry protocol (monitoring only)

USB CDC-to-UART on UNO 3 uses 115200 baud, ASCII newline-delimited lines. **The scanner has no serial command interpreter and never calls `Serial.read()` or `Serial.available()`**. Opening a Uno USB COM port can still automatically reset the microcontroller via DTR; that is a hardware/bootloader behavior, not a command from the GUI. The monitor only reads. The monitoring channel is intentionally independent of the chosen SPI wire coordinate mode.

| Line type | Fields after identifier | Interpretation |
|---|---|---|
| `HELLO` | `UNO3|SPI_MASTER|1` | Boot / protocol description |
| `CONFIG` | key + value | Configuration (`SPI_COORD_MODE`, `PULSES_PER_REV`, `SPI_HZ`, `ENA_FREE_LEVEL`, `SERIAL`) |
| `STATUS` | `ms|mode|state|homed|step|hall|dout|aout|tof|strikes|selected|count|torque|spiGood|spiBad|lastCmdSeq|coordMode|error|lastRangeMm|samples|invalidSamples|spiTimeout` | Scanner status every 250 ms, exact field order. Last command sequence=255 until first acceptance; `torque` is commanded, not measured. |
| `OBJECT` | `id|x_mm|y_mm|r_mm|theta_cdeg|vx_mm_s|vy_mm_s|vr_mm_s|vt_mm_s|quality|age|flags` | Both representations, on track update and ~750 ms refresh. `theta` is **centidegrees** as an integer. |
| `LOST` | `id` | Expired track, remove from PC table |
| `RESET` | `OBJECTS` | All tracks removed (calibration/recovery) |
| `CALIBRATION` | `not-detected`, `magnet-detected|5..1`, `reference-locked` | Manual adjustment and automatic locking feedback |
| `TORQUE` | `FREE` or `HOLD` | Driver ENA command changed |
| `MODE` | mode code, optional selected ID | Local accepted scan-mode transition |
| `ERROR` / `HALL_CHECK` / `LEG` / `CYCLE` / `TRACK` / `TARGET_LOST` / `INFO` / `TOF` | Diagnostic event-specific fields | Human-readable debugging context; not intended as UNO 1 control |

The GUI `uno3_spi_radar_gui.py` plots object Cartesian `(x,y)` in a 1000 mm square and tables `x,y,r,θ,vx,vy,vr,vt,quality,age`, along with calibration, torque command, ToF availability, SPI receive good/bad counts, command acknowledgment, selected target and mode, Hall strikes, latest range, sample counters and recent events. It sends **zero user commands**. It can simulate a labeled, non-hardware display with `--simulate`.

## 10. Integration and validation plan for other board creators

1. **Power off** all boards and the motor driver. Check stepper current limit, 1/16 DIP switch and ENA polarity with the exact TB6600-compatible module, then wire SPI and ground per table. A floating ENA during reset can be unsafe; no software setting guarantees de-energization during boot.
2. Build and flash **UNO 1 slave reference first**, then **UNO 3 scanner** (needs Pololu VL53L0X Arduino library); use the correct Uno/ATmega328P/16 MHz board selection. UNO 3 sketch's header files must be beside its `.ino` file.
3. On UNO 3, immediately after power-up observe `HELLO`, `CALIBRATION|not-detected`; confirm the motor shaft is physically free *in calibration only*, with safe mechanism. Rotate magnet into Hall active zone, hold steady five seconds, observe `TORQUE|HOLD` and `CALIBRATION|reference-locked`.
4. Confirm UNO 3 receives at least one correctly CRC-validated MISO command per ~100 ms (`SPI RX OK` rises; `SPI RX BAD` not continuously rising). If not, inspect common ground, CS, AVR slave SS input/output role, SPI mode and gap timings.
5. Using a real UNO 2 UART terminal/application, send `CMD|1|0` through UNO 1 to start slow scanning. Confirm the next UNO 3 telemetry ACK advances, active mode changes to 1, movement begins only if homed and ToF ready; observe 0→360→0 operation without tether damage.
6. Test Cartesian frame decoding for simple known target plates at `(750,500)`, `(500,750)`, `(250,500)`, `(500,250)` and `(700,700)` mm. Compare with the USB GUI; do not claim absolute millimeter accuracy without calibration trials.
7. Change `SPI_COORDINATE_MODE` to cylindrical, **recompile UNO 3**, confirm status bit 5 is set, and confirm a 300 mm target at (500,800) produces approximately `r=300`, `theta_cdeg=9000`. UNO 1 must interpret it without modification.
8. Send `CMD|3|<id>`; observe targeted ±30° visits and full refresh after four visits. Remove the target and verify loss flag and slow-search fallback. Test Hall drift detection and its three-strike recalibration flow.
9. Test command corruption (flip a byte without adjusting CRC), wrong version, bad mode, nonzero reserved bytes, truncated CS transaction and command sequence duplicates. Ensure no unsolicited mode execution, appropriate status flags/errors, and no byte indexing overflow.
10. Test SPI MISO disconnected **after** a valid link and while scanning: motion should cease after ~3 s, holding torque remain engaged outside calibration, home become invalid if stopped mid-sweep, and REHOME be required before restarting.
11. Use a logic analyzer to capture all 48 MOSI and MISO bytes, SS, SCK and relative timestamps under maximum expected UNO 1 UART and UNO 4 I²C activity. Verify first response byte, no CRC corruption, and that the motor STEP ISR remains responsive during SPI transactions.
12. Only after electrical, functional, timing and optical tests should you integrate the reference UNO 1 slave into the actual multi-board central hub firmware. Extend or version the protocol intentionally, with corresponding documentation and codec tests.

### Known boundaries and future extension policy

- The exact 48-byte protocol, error codes, retry flags, command timeout, UART output format and telemetry extensions at 42–46 are implementation decisions introduced here to complete an underspecified outline; they are **not** claimed hardware-tested requirements of the prototype.
- The scanner firmware currently accepts ToF distances from 40 mm up to but not including **500 mm**, inherited from the supplied prototype; targets near the approximately 707 mm scanner-to-corner distance are outside this default measurement gate unless `MAX_TARGET_MM` is deliberately recalibrated and changed.
- No global synchronous clock/time stamps are present in v1. Use local UNO 3 acquisition time for tracking and host arrival times for UI liveness. SPI telemetry is a best-effort *snapshot*, not a time-stamped measurement log.
- Current tracking is optimized for few widely separated static plates. The 3-object limit, velocity estimates, Hall edge check, sample age and cluster quality are not a hardware-accuracy certification.
- A future protocol may add time stamps, control request IDs, explicit fault reset, additional target slots, more precise coordinates or error diagnostics: **increment version** if any offset/unit/semantic changes, and maintain backwards-compatible parser tests.
- The `UNO1_SPI_Hub_Reference` cannot be assumed to coexist with arbitrary blocking I²C/UART code without ISR scheduling tests. Its intended value is an executable interoperable reference for UNO 1 / UNO 2 developers.

## 11. Files to use when implementing another peer

- `UNO3_SPI_Scanner/RadarProtocol.h` — shared constants, enum values, CRC, integer LE encode/decode helpers.
- `UNO1_SPI_Hub_Reference/RadarProtocol.h` — exact same header, copied because Arduino IDE sketch folders compile separately.
- `UNO1_SPI_Hub_Reference/UNO1_SPI_Hub_Reference.ino` — AVR SPI slave ISR, buffering, command staging, UART integration reference.
- `radar_spi_protocol.py` — portable Python frame generator/decoder for a logic-analyzer dump or host test.
- `tests/test_protocol.py` — CRC check vector, command framing, corrupted CRC rejection, both coordinate formats and PC serial parsers.
- `README_SPI.md` — installation, scope, assumptions and tests.

