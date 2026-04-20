# DIP Switch Settings

## SW1-SW4: Subdivision (Microstepping) Settings

When SW1-SW4 are all ON, the subdivision is set via software (P03-09 parameter).
When any are OFF, hardware settings override:

| Subdivision | SW1 | SW2 | SW3 | SW4 |
|-------------|-----|-----|-----|-----|
| Default (software) | ON | ON | ON | ON |
| 800 | OFF | ON | ON | ON |
| 1600 | ON | OFF | ON | ON |
| 3200 | OFF | OFF | ON | ON |
| 6400 | ON | ON | OFF | ON |
| 12800 | OFF | ON | OFF | ON |
| 25600 | ON | OFF | OFF | ON |
| 51200 | OFF | OFF | OFF | ON |
| 1000 | ON | ON | ON | OFF |
| 2000 | OFF | ON | ON | OFF |
| 4000 | ON | OFF | ON | OFF |
| 5000 | OFF | OFF | ON | OFF |
| 8000 | ON | ON | OFF | OFF |
| 10000 | OFF | ON | OFF | OFF |
| 20000 | ON | OFF | OFF | OFF |
| 40000 | OFF | OFF | OFF | OFF |

### OSSM Standard Setting

**SW1-SW4 all ON** (software controlled, P03-09 = 800 for OSSM compatibility).

The OSSM firmware expects 800 steps per revolution. This matches the official
OSSM firmware DIP switch documentation. Setting it via software (all ON) and
writing P03-09 = 800 via the JMC PC software is equivalent to OFF-ON-ON-ON.

## SW5: Input Edge Setting

| SW5 | Edge |
|-----|------|
| OFF | Rising edge (pulse counted on rising edge) |
| ON | Falling edge (pulse counted on falling edge) |

**OSSM**: OFF (rising edge) - matches ESP32 step output.

## SW6: Logical Direction Setting

| SW6 | Direction |
|-----|-----------|
| OFF | CCW = forward |
| ON | CW = forward (reversed) |

**OSSM**: Depends on physical mounting orientation.

## SW7: Pulse Filter (V5 firmware only, some models)

| SW7 | Filter |
|-----|--------|
| OFF | 200 kHz max (no filter) |
| ON | Lower frequency filter enabled |

**OSSM**: OFF (maximum pulse frequency for high-speed operation).

Note: Not all iHSV57 models have SW7. The V6 firmware uses P03-04 for pulse filtering.
