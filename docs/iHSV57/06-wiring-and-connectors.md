# Wiring & Connectors

## RS232 Serial Communication Port

5-pin spring terminal connector (top to bottom as shown on page 43 of V2 manual):

| Pin | Label | Wire Color | Description |
|-----|-------|------------|-------------|
| 1 | NC | - | Not connected (reserved) |
| 2 | RX | Brown/white | Receive data (connect to controller TX) |
| 3 | GND | Blue | Ground |
| 4 | TX | Blue/white | Transmit data (connect to controller RX) |
| 5 | VCC | - | 3.3V power output (for HISU text display, do NOT use for general power) |

### Voltage Levels: The Great Debate

There is conflicting information in the community about whether the RS232 port
uses true RS232 voltage levels or TTL levels:

**Sources saying RS232 levels (needs level shifter):**
- iHSV-Servo-Tool README: "3.3V logic will NOT work directly"
- Community reports: PL2303 USB-RS232 cables work (these have RS232 level output)
- MikesMachines: "Trying a MAX3232 chip for TTL to RS232"

**Sources suggesting TTL levels (might work direct):**
- Tuning software research: "The motor's RS232 port uses TTL voltage levels (0-3.3V or 0-5V)"
- Some users report success with TTL serial adapters
- The VCC pin outputs 3.3V (suggesting 3.3V logic internally)

**Recommendation:** Start with a MAX3232 level shifter to be safe. If that doesn't
work, try direct 3.3V connection. The safest approach for ESP32 is:

```
ESP32 TX (GPIO) -> MAX3232 T1IN -> MAX3232 T1OUT -> Servo RX (pin 2)
Servo TX (pin 4) -> MAX3232 R1IN -> MAX3232 R1OUT -> ESP32 RX (GPIO)
ESP32 GND <-> MAX3232 GND <-> Servo GND (pin 3)
```

### USB-to-Serial Adapters (for PC connection)

| Chipset | Status |
|---------|--------|
| PL2303 | **Recommended** - The PL2303 chip is inside the DB9 connector end of most USB-RS232 cables. Install drivers BEFORE plugging in. Watch for counterfeit chips. |
| CH340 | Works for some users |
| FTDI | **Reportedly does NOT work** with iHSV57 |

**DB9 pin mapping** (for USB-RS232 cable with DB9 female end):
- Pin 2 (RXD) -> Servo TX (pin 4)
- Pin 3 (TXD) -> Servo RX (pin 2)
- Pin 5 (GND) -> Servo GND (pin 3)

**Important**: Some sources say TX-to-TX, RX-to-RX (not crossover). Test both
if you're having trouble. The correct wiring depends on whether the source labels
pins from the servo's perspective or the cable's perspective.

### COM Port Note (Windows)

Use COM3 through COM8 only. The JMC software stops scanning above COM8.
COM1 and COM2 may have conflicts. Change port numbering in Device Manager
under the driver's advanced port properties.

---

## Control Signal Input Port

2-position spring terminal connector (x3 pairs):

| Symbol | Function | Low (valid) | High (valid) |
|--------|----------|-------------|--------------|
| PUL- | Step pulse - | 0-0.5V | - |
| PUL+ | Step pulse + | - | 4-5V |
| DIR- | Direction - | 0-0.5V | - |
| DIR+ | Direction + | - | 4-5V |
| ENA- | Enable - | 0-0.5V | - |
| ENA+ | Enable + | - | 4-5V |

### Opto-isolated inputs

All control signal inputs are opto-isolated. VCC is compatible with **5V-24V**.
For 3.3V signals (ESP32), a series resistor of 3-5K ohm may be needed on
the common terminal.

### Wiring Modes

1. **Common anode** (VCC to PUL+/DIR+/ENA+, signals pull low): Standard for most controllers
2. **Common cathode** (GND to PUL-/DIR-/ENA-, signals push high): Alternative
3. **Differential** (both +/- driven): Best noise immunity

### Timing Requirements (from page 44 of manual)

```
t1 > 5us   ENA must be stable 5us before first pulse
t2 > 6us   DIR must be stable 6us before pulse edge
t3 > 2.5us Pulse width (high time) minimum 2.5us
t4 > 2.5us Pulse low time minimum 2.5us
t5 > 5us   ENA setup time
```

High level threshold: > 3.5V
Low level threshold: < 0.5V (manual says 3.5V for low level boundary)

---

## Alarm Output Port

2-pin spring terminal:

| Symbol | Function |
|--------|----------|
| ALM- | Alarm output - (open collector) |
| ALM+ | Alarm output + (opto-isolated) |

Active when servo is in alarm state. Wire into E-stop loop for safety.

## Position Complete Output Port

2-pin spring terminal:

| Symbol | Function |
|--------|----------|
| PED- | Position end detection - |
| PED+ | Position end detection + |

Active when position command is complete (within P03-06 tolerance).

## Power Input

| Symbol | Function | Notes |
|--------|----------|-------|
| DC+ | Power + | 20-80V DC (select by model, 36V for iHSV57) |
| GND | Power - | |

**WARNING**: High input voltage WILL burn out the servo drive motor.
Select voltage according to model specifications:
- iHSV57-30-xx-36: 24-48V DC (50V absolute max)

---

## Encoder Output (Differential)

The motor has a differential quadrature encoder output compatible with 26LS32.
This is primarily used for external feedback in CNC controllers (LinuxCNC, Mach3, etc.)
and is NOT typically used in OSSM builds.

| Signal | Description |
|--------|-------------|
| A+ / A- | Channel A differential |
| B+ / B- | Channel B differential |
| Z+ / Z- | Index pulse differential |
