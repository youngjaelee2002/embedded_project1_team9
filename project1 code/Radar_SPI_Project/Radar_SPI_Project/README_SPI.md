# Arduino four-board scanner — SPI migration delivery

This package converts the user-supplied **standalone UNO 3 prototype** into a scanner-controlled-over-SPI architecture. Its scanner maintains the prior Timer1 nonblocking step motor, Pololu VL53L0X acquisition, Hall five-second manual zero calibration, 1/16 microstep assumption, distance clustering, object ID association, target search/tracking, and three-strike Hall drift flow. It differs primarily in command transportation, automatically managed torque, and the PC status interface.

## Deliverables

- `UNO3_SPI_Scanner/UNO3_SPI_Scanner.ino` — **UNO 3 SPI master**, motion, measurement, tracking, automatic torque, binary telemetry, read-only USB monitoring.
- `UNO3_SPI_Scanner/UNO3_Types.h` — custom types for Arduino's `.ino` prototype preprocessor.
- `UNO3_SPI_Scanner/RadarProtocol.h` — wire constants/CRC with no ABI struct layout assumptions.
- `UNO1_SPI_Hub_Reference/UNO1_SPI_Hub_Reference.ino` and its `RadarProtocol.h` — reference **UNO 1 SPI slave** and rudimentary UNO 2 UART bridge. **Not a complete UNO 1 central hub.**
- `uno3_spi_radar_gui.py` — Tkinter/pyserial **read-only** local USB monitoring, radar plot, Cartesian+cylindrical objects, CSV and simulation.
- `SPI_PROTOCOL.md` — full 48-byte wire layout, timing, CRC, command ACK, error semantics, coordinate conversions, safety/recovery and testing.
- `radar_spi_protocol.py`, `tests/test_protocol.py` — independent Python protocol reference and unit tests.
- `ORIGINAL_OUTLINE.md`, `ORIGINAL_STANDALONE.ino` — unmodified historical user-provided reference snapshots (not the active firmware).

## Setup

1. Install Arduino IDE, select **Arduino Uno R3 (ATmega328P, 16 MHz)**, and install the **Pololu VL53L0X Arduino library** for the UNO 3 build. These are two **separate Arduino sketches** in their respective folders. Keep each `.ino` together with its `RadarProtocol.h` (and `UNO3_Types.h` for UNO 3).
2. Verify the original wiring and motor supply/current restrictions before energizing. Common-anode motor driver: D2=PUL−, D3=DIR−, D4=ENA−, Hall D7/A0, sensor A4/A5. **Verify whether ENA LOW means FREE** on your actual TB6600 driver. If opposite, change `ENA_FREE_LEVEL` near the top of UNO 3 sketch. The motor must *physically* release during CALIBRATION and hold after lock; no firmware guarantee for reset transients.
3. New SPI wiring: UNO 3 D9→UNO 1 D10, D11↔D11, D12↔D12, D13↔D13, GND↔GND. UNO 3 D10 is OUTPUT/HIGH and **does not connect** to UNO 1. UNO 1 UART D0/D1 connects to UNO 2 UART, at compatible logic voltages and common ground, when required. If UNO1 UART is connected externally, ensure other attached USB serial circuitry doesn't drive conflicting UART lines.
4. To choose **Cartesian SPI slots** leave `SPI_COORDINATE_MODE = RadarWire::COORD_CARTESIAN;`; to use **cylindrical** replace with `RadarWire::COORD_CYLINDRICAL;` in `UNO3_SPI_Scanner.ino`, then re-upload. The GUI **always** displays both representations.
5. Upload UNO 1 reference firmware (or your fully integrated compatible UNO 1 implementation). Upload UNO 3. Manual five-second Hall calibration releases motor and restores hold automatically; it can then be commanded by UNO 1. UNO 3 has **no local serial command interface**.
6. `python -m pip install -r requirements.txt` then `python uno3_spi_radar_gui.py`. Connect the **UNO 3 USB** COM port and view status/objects. `python uno3_spi_radar_gui.py --simulate` is a labeled UI demo, not proof of hardware functionality.
7. To request a mode via the reference UNO 1 from the UNO 2 UART side, send e.g. `CMD|1|0\n` for SEARCH_SLOW, `CMD|2|0\n` for FAST, `CMD|3|2\n` for TRACK_TARGET ID 2, `CMD|0|0\n` for STOP, or `CMD|4|0\n` for REHOME. To request a ToF retry when not moving send `RETRY\n`. The UART routing is UNO 2→UNO 1, **not** PC→UNO 3.

## Safety notes

- The motor driver supply comes from the external motor supply, **never Uno 5V**. Do not wire or unplug motor coils while powered. Confirm driver current-limit / peak-vs-RMS semantics and 1/16 microstep setting; a 3.5 A DIP setting for an unsuitable motor can damage it.
- Driver ENA polarity is module-specific. Software writes an **ENA command**, not a torque measurement. A reset/USB connection can transiently change outputs; no guaranteed safe torque state exists until MCU firmware initializes.
- Maintain 0→360→0 tether-safe motion. Confirm clearance and flexible sensor cable travel without motor power first. REHOME relies on **manual** Hall alignment and a steady five-second detection, not an actively rotating motor.
- The scanner's ToF and motor functions came from supplied prototype sources, but **the new SPI code and GUI have not been physically tested**. The included UNO 1 reference is not the final hub with I²C turret functionality. Arduino AVR compilers/toolchains may differ; bench testing remains mandatory.

## Checks without hardware

```bash
python -m py_compile uno3_spi_radar_gui.py radar_spi_protocol.py
python -m unittest discover -s tests -v
```

The Python unit suite exercises CRC-8/ATM (`123456789`→`F4`), command frames, frame corruption, Cartesian and cylindrical decoding, and the monitoring parsers. See `SPI_PROTOCOL.md` for a hardware integration test checklist. **Passing these software checks is not equivalent to flashing and validating firmware on physical Uno boards.**

## Status / limits

This is **scanner-focused**, matching the outline's scope. UNO 2's final GUI protocol, UNO 1's full UART/I²C application, and UNO 4's turret code are not specified in the supplied prototype, so the UNO 1 implementation here is an interoperability reference rather than a complete 4-board application. Nominal 0.5/1 rev/s peaks and VL53L0X 20 ms budget are carried from the prototype; actual motion accuracy/measurement accuracy remain to be measured. See SPI protocol documentation for command timeout (3 s) and other newly fixed design choices.


## Additional Changes (Post-Active Testing)

CRC to any non-hardware required testing has been verified and checked. Any remaining tasks are applying UNO 3 adaptation to the other team's UNO boards. Verify the additional SPI related protocol + structure that has been addtionally defined and proceed with finalization of the project.