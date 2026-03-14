# iHSV57 Overview & Specifications

## Model Numbering

`iHSV57-30-18-36-21-38-SC-EC`

| Segment | Meaning |
|---------|---------|
| iHSV | Integrated High-performance Servo (AC) |
| 57 | NEMA frame size (57mm flange = NEMA 23) |
| 30 | Series/motor length identifier |
| 18 | Power rating in tens of watts (10=100W, 14=140W, 18=180W) |
| 36 | Rated voltage (36V DC) |
| 21 | Shaft length in mm (optional) |
| 38 | Pilot/centric diameter in mm (optional) |
| SC | Suffix: with brake |
| EC | Suffix: EtherCAT fieldbus |
| RC | Suffix: CANopen fieldbus |

## Electrical Specifications (iHSV57-30-18-36)

| Parameter | Value |
|-----------|-------|
| Rated power | 180W |
| Rated torque | 0.57 Nm (continuous, flat to 3000 RPM) |
| Peak torque | ~0.6 Nm |
| Rated speed | 3000 RPM |
| Max speed | 4200 RPM |
| Rated voltage | 36V DC |
| Operating voltage | 24-48V DC (50V absolute max) |
| Rated current | 7.5A |
| Max current | 15A (peak 18A) |
| Encoder | Optical, 1000 CPR (4000 counts/rev quadrature) |
| Max pulse frequency | 200 kHz |
| Default communication | 9.6 Kbps (additional interface required) |

## Control Loop Bandwidths

| Loop | Bandwidth |
|------|-----------|
| Current loop | 2 kHz |
| Speed loop | 500 Hz |
| Position loop | 200 Hz |

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Flange | NEMA 23 (57x57mm) |
| Mounting holes | 47.14mm spacing, 4x M5 |
| Shaft diameter | 8mm (D-shaft) |
| Shaft length | 21mm (standard) |
| Total length | 150mm (180W model) |
| Weight | ~1.6 kg |
| Working temperature | 0 to +70C |
| Storage temperature | -20 to +80C |
| Humidity | 40-90% RH |
| Cooling | Natural or forced air |

## Available Variants in NEMA 23 (57mm)

| Model | Power | Torque | Current | Length |
|-------|-------|--------|---------|--------|
| iHSV57-30-10-36 | 100W | 0.29 Nm | 3.5A | 110mm |
| iHSV57-30-14-36 | 140W | 0.44 Nm | 5.4A | 130mm |
| iHSV57-30-18-36 | 180W | 0.57 Nm | 7.5A | 150mm |

All share: 36V rated, 24-48V operating, 3000 RPM rated, 4200 RPM max.

## Communication Interfaces

| Interface | Description |
|-----------|-------------|
| Step/Direction | PUL+/-, DIR+/-, ENA+/- (opto-isolated, 5V-24V) |
| RS232 serial | 5-pin spring terminal (Modbus RTU, 57600 default) |
| Alarm output | ALM+/- (opto-isolated, open collector) |
| Position complete | PED+/- (signals motion complete) |

### Fieldbus Variants (different models)

| Suffix | Interface |
|--------|-----------|
| -EC | EtherCAT (250us sync cycle) |
| -RC | CANopen + RS485 |
| (none) | RS232 only |

## Protections

- Overcurrent (AL.063)
- I2T overload (AL.410/412)
- Over-voltage (AL.402)
- Under-voltage (AL.401)
- Overheat (AL.440/441)
- Overspeed (AL.420)
- Position error excessive (AL.501)
- Encoder fault (AL.600/610/611)
- EEPROM anomaly (AL.051)
- Serial communication exception (AL.943)

## Firmware Versions

| Version | Notes |
|---------|-------|
| V5xx | Older firmware, no auto-tune, cannot be updated, 8N1 serial default |
| V6xx | Current, has auto-tuning (P01-03), 8E1 serial default, updatable |
| v604 | Previous shipping version |
| v605 | Current shipping version (as of early 2025) |
| v606/v607 | Mentioned in some listings, no changelog found |

**Critical**: V5 and V6 firmwares have DIFFERENT Modbus register address maps.
The V5 register addresses (used by go-servoc) are NOT the same as V6.
See [Register Map](03-register-map.md) for both.
