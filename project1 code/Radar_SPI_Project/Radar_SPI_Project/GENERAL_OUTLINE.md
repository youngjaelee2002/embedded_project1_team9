# Arduino 4-Board Distributed Communication Project — Updated Handoff v2.0

**Focus:** UNO 3 — rotating 2D ToF scanner / mock radar subsystem
**Status:** Design and preliminary firmware prepared; **a few physical systems has been tested, ToF yet to be tested**.  
**Companion Arduino sketch:** [`UNO3_Motor_Hall_Bringup.ino`](UNO3_Motor_Hall_Bringup.ino) (the complete source is also included in Appendix A).

> Code Implementation Style: The final embedded systems code for the UNO 3 should be a "Hybrid Arduino / register-level embedded C++ firmware with interrupt-driven control and a cooperative state machine" rather than relying on higher-level arduino API functions. Organize code into blocks that allow ease of understanding, with minimal (just enough) commentation for code readability. Any code that will need to referenced will be included within the chat - a similar (last prototype) target-level code set and a higher-level test/example code for the hardware that has been verified for functionality. (Important to base the functions from the test code and turn them + implement them as lower-level code.)

## 1. Project goal and four-board topology

Implement **four Arduino Uno boards**. UNO 1 is the central hub and must exercise **UART, I²C, and SPI** in the overall project. The other boards each have local responsibilities.

```text
                   UNO 2 — GUI/interface
                          |
                         UART
                          |
                  UNO 1 — CENTRAL HUB
                    /             \
                  I²C             SPI
                 /                 \
        UNO 4 — Turret        UNO 3 — Scanner
                                (SPI master)
```

- **UNO 1:** central hub; UART endpoint to UNO 2, I²C endpoint to UNO 4, **SPI slave** to UNO 3. Other UNO 1 tasks and command policies remain to be implemented.
- **UNO 2:** graphical/user interface, communicating with UNO 1 over UART.
- **UNO 3:** local motor, ToF measurement, angle homing, object detection, target tracking, and **initiates SPI transactions** to UNO 1.
- **UNO 4:** turret subsystem, connected to UNO 1 over I²C; behavior not specified in this scanner-focused handoff.

UNO 3 should send **processed object/state data, not all raw ToF samples**. UNO 1's role as an SPI slave is an intentional **project requirement**, not an optimization recommendation.

## 2. Confirmed design decisions and current assumptions

| Item | Decision / status |
|---|---|
| Scanner Arduino | Arduino Uno R3-class ATmega328P target |
| Scanner sensor | VL53L0X ToF on local UNO 3 I²C pins A4/A5 |
| VL53L0X breakout electrical connections | User reports its **5 V, SDA and SCL** wiring compatibility already verified |
| Stepper | Planned 17HS4023 bipolar stepper, ~200 full steps/revolution|
| Motor driver | Planned TB6600-class STEP/DIR driver|
| Hall reference | **Waveshare Hall Sensor, SKU 9522**, 49E element plus LM393 comparator; replaces proposed bare `49E140BG` sensor due to availability |
| Hall data | Prefer module DOUT for a thresholded home event; connect AOUT for diagnosis |
| Rotation mechanics | Back-and-forth full turn, not indefinitely winding in one direction |
| Scan area | Horizontal open plane approximately 1,000 × 1,000 mm |
| Scanner location | Approximately (500, 500) mm at plane center |
| Scanner mounting | ToF sensing aperture at about 50 mm above plane, pointing horizontally; magnet suspended below head for fixed Hall pickup |
| Demonstration targets | Initially plates approximately **50 mm wide × 100 mm tall**, kept facing the scanner |
| Target count / merging | Few widely spaced objects; complex optical merging is out of scope for initial demo |
| Main quality priority | Reliable classroom demo and reasonably accurate coordinates for a **single** detected target |
| Velocity estimation | Desirable, secondary to stable positions and IDs |
| SPI role | **UNO 3 is master; UNO 1 is slave**; maintain full-duplex command/telemetry design |

## 3. Functional requirements

1. Establish a repeatable angular origin using the Hall module and a rotor-mounted magnet.
2. Rotate the ToF sensor continuously *during each sweep* and read distance while moving (do **not** pause the motor for every sample).
3. Implement `SEARCH_SLOW`, `SEARCH_FAST`, and `TRACK_TARGET` operating modes.
4. Use the step-count angle and a measurement-time correction to associate each ToF sample with a bearing.
5. Identify clusters of neighboring returns, estimate object position, and assign stable integer IDs (1, 2, 3, ...).
6. Send compact object records and scanner status to UNO 1 over SPI, initiated by UNO 3.
7. Receive desired scan mode and priority target ID from UNO 1 through the same SPI transfers.
8. Recover gracefully from target loss, missing Hall transitions, and unavailable/invalid ToF returns.
9. Eventually perform periodic Hall-reference checks on full sweeps and detect implausible angular drift.

**Not in initial scope:** precision angular metrology, resolving overlapping objects at the limit of the ToF FoV, advanced multi-object tracking, guaranteed performance at 1 rev/s, or pre-tested production firmware.

## 4. Proposed hardware and pin map

### 4.1 Parts

- 1 × Arduino Uno R3-compatible board (UNO 3).
- 1 × VL53L0X breakout (user-verified breakout-level 5 V/I²C connections).
- 1 × 17HS4023 bipolar stepper or equivalent with verified winding/phase ratings.
- 1 × TB6600-compatible microstepping STEP/DIR driver (exact version TBD).
- 1 × [Waveshare Hall Sensor](https://www.waveshare.com/hall-sensor.htm) (SKU 9522; 49E + LM393, AOUT + DOUT).
- 1 × small neodymium magnet, secure rotor attachment.
- Appropriate mounting bracket, short flexible ToF cable, motor coupler/shaft mounting, and adjustable laboratory DC supply.
- PC USB connection and jumper wires; an appropriate logic interface/driver may be required if TB6600 input current exceeds the Uno's safe GPIO capability.

### 4.2 UNO 3 pin assignments

| Arduino Uno pin | Destination | Notes |
|---|---|---|
| D2 | TB6600 `PUL−` | Step/pulse, common-anode wiring assumption |
| D3 | TB6600 `DIR−` | Direction, common-anode wiring assumption |
| D4 | Optional TB6600 `ENA−` | **Leave disconnected in first test**; enable polarity/module behavior varies |
| D7 | Waveshare Hall `DOUT` | Digital home threshold; selected instead of Waveshare example D2, because D2 drives motor |
| A0 | Waveshare Hall `AOUT` | Calibration/diagnostic analog value |
| A4 | VL53L0X `SDA` | Local scanner I²C |
| A5 | VL53L0X `SCL` | Local scanner I²C |
| D9 | **UNO 1 D10 / SS** | UNO 3's SPI **slave-select output** for UNO 1 |
| D10 | **Configured OUTPUT, driven HIGH** | UNO 3 hardware SS pin must remain output to retain SPI master role; not connected to UNO 1 for this design |
| D11 | UNO 1 D11 / MOSI | SPI data **from UNO 3** to UNO 1 |
| D12 | UNO 1 D12 / MISO | SPI data **from UNO 1** to UNO 3 |
| D13 | UNO 1 D13 / SCK | SPI clock **from UNO 3** |
| D0/D1 | USB serial hardware | Used in standalone debug sketches; avoid additional external UART load on UNO 3 during USB debugging |
| D5, D6, D8 | Available | May be used for data-ready, safety, or other functions later |

UNO 1 SPI slave uses D10 SS as an **input** (pulled low by UNO 3 D9 during a transfer), D11 MOSI input, D12 MISO output, and D13 SCK input. The SPI bus between UNO 1 and UNO 3 requires a **shared signal ground**. Arduino Uno 1's UART pins D0/D1 to UNO 2 and A4/A5 I²C to UNO 4 are physically separate from its SPI pins, but firmware must still manage time and interrupt latency.

### 4.3 Waveshare Hall module wiring

```text
Waveshare Hall         UNO 3
-------------         -----
VCC  ----------------> 5V
GND  ----------------> GND
DOUT ----------------> D7       (thresholded Hall status)
AOUT ----------------> A0       (analog diagnostic signal)
```

### 4.4 TB6600-like driver wiring

```text
UNO 3 5V ---------> TB6600 PUL+
UNO 3 D2 ---------> TB6600 PUL-
UNO 3 5V ---------> TB6600 DIR+
UNO 3 D3 ---------> TB6600 DIR-
TB6600 ENA+/ENA- -> leave disconnected pending module verification

Bench supply + ----> TB6600 motor supply positive terminal
Bench supply - ----> TB6600 motor supply negative terminal
17HS4023 phase A --> TB6600 A+/A-
17HS4023 phase B --> TB6600 B+/B-
```

### 4.5 Microstepping and old DIP values

Design starting point is **1/16 microstepping**

**-> Along with this handoff will be included several test codes that have been verified to work (test_{module}.ino). Use that as the basis of operation.**

## 5. Physical layout, magnet, and cable routingf (To be Physically determined by user, use this description as a reference)

Place motor rotation axis close to the plane center `(500,500)` mm. Mount the VL53L0X approximately **50 mm above the tabletop/plane**, aimed horizontally along the rotating head's radial direction. Nominal target plates are **50 mm wide and 100 mm high**; their vertical center lies at roughly 50 mm, aligning with the sensor if the plate stands flush on the plane.

Mount one small magnet rigidly **below the rotating head**. Mount the Waveshare Hall module on a fixed nonrotating bracket near the magnet's circular path. Adjust magnet radius, air gap and sensitivity so DOUT is triggered within a relatively short angular region, producing one useful reference edge per full rotation. Use a **specific inactive-to-active edge in the same positive travel direction** when defining home; do not define reference from a sustained HIGH or LOW signal alone.

Because the sensor cable is tethered, use repeated **0° → 360° → 0°** sweeps, with adequate slack and a hard mechanical clearance margin. Do not spin several turns in the same direction. Verify the rotor cannot snag its cable, Hall bracket, nearby people, or target plates.

Note that **50 mm of height does not guarantee optical isolation from the tabletop**. At an approximately 25° nominal VL53L0X FoV and 500 mm range, the angular cone spans roughly 220 mm across. A 50 × 100 mm target fills only part of that field. Test reflective white/matte plates and background conditions; reliable readings and position accuracy are not guaranteed solely by placing the sensor at mid-height.

## 6. Startup home calibration procedure

-> "CALIBRATION"

Any actual adjustments to home(0 degrees) location will be done by hand. This procedure (state) should be intiated during the start-up phase of the radar and if a threshold on the hall-sensor rotation return check has exceeded a threshold (3-strike).

1. Squawk Serial "CALIBRATION|not-detected"
2. Cease all operation of the motor (Motor should be turable by hand)
3. The operator will align the radar via hand. When the motor is close to the reference point, the hall sensor will read magnet detected. Start a countdown for 5 seconds and squawk "CALIBRATION|magnet-detected|5" (countdown to) "CALIBRATION|magnet-detected|1"
4. After the countdown, lock the current position as 0 degrees reference point. squawk "CALIBRATION|reference-locked"

**Squawked serial codes can be changed accordingly, as will be mentioned in '10. Debugging procedure. Wording or formats can be changed, just be sure to make the procedure similar to the procedure mentioned.**


## 7. Scan modes and motor state machine

```text
  (system power on)
         |
         v
     CALIBRATION -- (reference locked) -->  IDLE (until input from UNO 1)
                                              |
                                              +--> SEARCH_SLOW
                                              +--> SEARCH_FAST
                                              +--> TRACK_TARGET

During tracking modes (SEARCH, TRACK), each state can be changed to each other via UNO 1's command request
Also, during SEARCH,TRACK -> hall-sensor rotation return check threshold exceeded (failed home / motor fault)
-> Intiate CALIBRATION state again
-> Once calibrated and reference is homed, return back to previous action
```

### 7.1 SEARCH_SLOW and SEARCH_FAST

- `SEARCH_SLOW`: full 0→360→0 sweep, intended peak **0.5 rev/s**.
- `SEARCH_FAST`: same motion, intended peak **1 rev/s**.

### 7.2 TRACK_TARGET

- UNO 1 commands mode `TRACK_TARGET` and one selected object ID (e.g., object 1).
- UNO 3 locates the newest valid bearing for this ID and chooses a narrow range around it, initially **±30°**, clipped and routed within cable-safe motion limits.
- Do approximately **4 narrow back-and-forth sweeps**; then one **full fast** sweep for awareness and a chance to cross the Hall reference in the calibrated direction. Repeat.
- If the selected object has been absent for several observations, return to `SEARCH_SLOW` or `SEARCH_FAST` (a project-configurable fallback) and report `TARGET_LOST`.
- The four narrow sweeps and ±30° are proposed defaults, **not experimentally selected constants**.

### 7.3 Integrated motor firmware recommendation

Use a timer-/scheduler-driven STEP generator or a verified nonblocking stepping library, and keep VL53L0X acquisition and SPI service independent of long motor-moving loops. The separate first-test `.ino` deliberately uses blocking motion to isolate hardware; **do not treat that as the final integrated firmware architecture**.

## 8. ToF acquisition, angular calibration and single-target accuracy

### 8.1 Distance and bearing

A full turn is 16 microsteps (3200 pulses/rev):

This is *commanded* motor angle, not encoder-measured shaft angle. Missed steps are possible.

During each ToF measurement, record an approximate start timestamp and end timestamp, plus the estimated motor angle. As a first approximation, associate the distance with the **angle at the midpoint of the integration/measurement interval**. Later calibration should include constant sensor latency and any rotating-head offset from the physical motor axis.

The VL53L0X commonly uses a **33 ms** timing budget; an approximately **20 ms** minimum is available at a performance tradeoff. The typical specified FoV is around **25°**, much wider than 0.225° commanded microsteps. Approximate ideal sample-center spacing is:

| Peak rotation rate | 33 ms measurement budget | 20 ms budget |
|---|---:|---:|
| 0.5 rev/s | 5.94° | 3.6° |
| 1 rev/s | 11.88° | 7.2° |

These calculations are only *timing-based sampling spacings*. Actual distance samples have overhead, reflectivity limits, range errors and broad optical reception; they are **not** achieved angular accuracy.

### 8.2 Board/world coordinates

Coordinate convention: top view, `x` grows right, `y` grows up, origin `(0,0)` at the board's lower-left corner. Scanner axis approximately `(500,500)` mm. Zero bearing points along +X; positive angle is counterclockwise.

```text
x_mm = 500 + (r_mm + radialOffset_mm) * cos(theta_world)
y_mm = 500 + (r_mm + radialOffset_mm) * sin(theta_world)
```

Here `radialOffset_mm` is zero **unless** the ToF measurement origin is radially displaced from the rotation axis; measure and calibrate this offset later. For a nonradial mount, use a full 2D head-to-axis transform rather than only a radial scalar offset. Clip or flag world coordinates beyond the 0–1000 mm board limits rather than silently treating them as valid.

Center-to-corner distance is `sqrt(500² + 500²) ≈ 707 mm`. Whether a **particular 50 × 100 mm target** is reliably detected at this distance must be tested; nominal sensor range claims under full-field target conditions do not guarantee that a smaller target at a corner is equally reliable.

### 8.3 Proposed object detection pipeline (favor accuracy for one object)

```text
VL53L0X measurement + time + estimated bearing
    -> check distance validity / range bounds
    -> optional per-angle empty-scene baseline and background rejection
    -> reject isolated outliers
    -> cluster adjacent angular samples with similar range
    -> locate cluster center angle (first/last accepted bearing midpoint)
    -> robust cluster range (median, with basic quality flags)
    -> convert to x/y and apply calibrated zero/radial offset
    -> associate with existing object ID, else allocate new ID
    -> publish latest object snapshot over SPI
```

The **first/last detection angle midpoint** is a useful starting estimate for a face-on target's center, if detection is reasonably symmetric. It is not a guaranteed optical centroid. Compare forward and backward sweeps of a static plate, estimate consistent angular bias with known test positions, and average or median repeated detections if that improves actual results.

**Simple baseline calibration target plan:** put a single plate at known positions, e.g. `(750,500)`, `(500,750)`, `(250,500)`, `(500,250)`, and a diagonal near `(700,700)`, then record the mean and spread of estimated x/y from multiple slow sweeps. Do not claim specific millimeter accuracy until these physical trials are conducted.

### 8.4 Multi-object IDs and optional velocity

For a few well-separated plates, a simple nearest-neighbor tracker is sufficient as a first attempt. Carry forward an ID if the new x/y lies within a physically plausible matching radius and another track is not a better candidate; otherwise allocate a new ID. Expire IDs only after a configurable number of missed scans to avoid flickering IDs. Track `lastSeenMs` and number of detections.

Optional velocity:

```text
vx_mm_per_s = (x_new - x_previous) / delta_t_seconds
vy_mm_per_s = (y_new - y_previous) / delta_t_seconds
```

Smooth coordinates before taking derivatives and mark velocity invalid until two useful position observations exist. Velocity is **not required** for the first demonstration.

## 9. SPI transport: UNO 3 master, UNO 1 slave

### 9.1 Physical wiring

```text
UNO 3 (MASTER)            UNO 1 (SLAVE)
D9  (GPIO CS output) ---> D10 (hardware SS input)
D11 (MOSI)          ---> D11 (MOSI input)
D12 (MISO input)    <--- D12 (MISO output)
D13 (SCK output)    ---> D13 (SCK input)
GND                  --- GND

UNO 3: also configure its OWN D10 as OUTPUT and hold it HIGH.
```

**SPI is full-duplex:** UNO 3 can clock out an object packet **and** simultaneously clock in the next pending UNO 1 command. UNO 1 cannot initiate an SPI transaction while it is the slave, so its command becomes actionable only when UNO 3 initiates the next transfer. This is an intentional consequence of the selected topology.

### 9.2 Proposed protocol

Prefer small, fixed-size packets and fixed-size arrays on the ATmega328P. One possible starting point is a **48-byte full-duplex frame** in both directions, with the same structural header and an 8-bit CRC (polynomial and initialization to be fixed during SPI implementation).

| Byte(s) | UNO 3 telemetry, sent over MOSI | UNO 1 command, sent over MISO |
|---|---|---|
| 0 | Sync marker `0xA5` | Sync marker `0xA5` |
| 1 | Protocol version `0x01` | Protocol version `0x01` |
| 2 | Scan sequence counter | Command sequence counter |
| 3 | Active mode | Requested mode |
| 4 | Scanner status bits | Command flags / stop request |
| 5 | Number of valid objects | Selected target ID |
| 6–41 | Three 12-byte object slots | Reserved; possible parameters later |
| 42–46 | Reserved/status expansion | Reserved |
| 47 | CRC-8 over bytes 0–46 | CRC-8 over bytes 0–46 |

**Each 12-byte object slot, Cartesian:** `id:u8`, `flags:u8`, `x_mm:i16`, `y_mm:i16`, `vx_mm_s:i16`, `vy_mm_s:i16`, `quality:u8`, `age_scans:u8`, all multibyte integers **little-endian**. Coordinates within the board are normally 0–1000 mm; signed fields permit out-of-bounds intermediate data to be represented and flagged. Three tracks per packet is a **demonstration capacity**, not a hard mathematical limit. Target IDs begin at 1; ID 0 can mean no target.

**Another 12-byte object slot, Cylindrical** `id:u8`, `flags:u8`, `r_mm:u16`, `theta_cdeg:u16`, `vr_mm_s:i16`, `vtheta_mm_s:i16`, `quality:u8`, `age_scans:u8`, same bases as the previous object slot, adjusted to fit a cylindrical coordinate procedure. Units are either mm (r_mm), degrees (theta_cdeg), and mm/s (vr_mm_s and vtheta_mm_s) with vtheta_mm_s being a tangential velocity. **Program the code so that both 12-byte slots are ready within the code itself and interchangable by changing a simple user-defined variable's value within the start of the .ino code.**

Proposed mode codes: `0=STOP/IDLE`, `1=SEARCH_SLOW`, `2=SEARCH_FAST`, `3=TRACK_TARGET`, `4=REHOME`. Reserve other codes. Target commands must be validated; on invalid frames retain safe last-known settings and report an error. Sequence numbers make stale transfers detectable. The ultimate command timeout/safety policy is TBD.

**Important UNO 1 slave implementation detail:** SPI byte responses need to be placed into `SPDR` in time, and the first response byte must be available when selection starts. Use a minimal SPI transfer ISR and prepared transmit buffer; copy completed received data into a main-loop buffer. Begin conservatively at **125 kHz SPI clock** with **interbyte gaps** and adequate time between frames while UNO 1 also handles UART and I²C. Timing/ISR reliability must be verified on the exact UNO 1 firmware; don't assume 48 back-to-back bytes are lossless under concurrent interrupts.

For now, **do not wire SPI during the standalone motor/Hall test**. Test UNO 3↔UNO 1 communication only after local motor and sensor bring-up.

### 10. Debugging and External (Serial) Visualization Procedure

**As done in CALIBRATION mode**, the Arduino Uno 3 will be connected to serial via USB in order to monitor during debugging procedures and also a possible python program to visualize the actual radar detection on a PC interface (GUI). Whilst no specific format exists at the current stage of development, during creation of the UNO 3's code, assign and implement an appropriate serial communication that allows both debugging and possible PC GUI implementation. The formats are not fixed to those mentioned in '6' and can be changed accordingly.
