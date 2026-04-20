# Alarm Codes

## LED Flash Codes

Red LED flashes with 0.8s on time, 2s interval between flash groups.

| Flashes | Alarm | Description | Action |
|---------|-------|-------------|--------|
| 2 | Overcurrent | Short circuit in motor UVW | Check motor wiring |
| 3 | Position deviation | Position error exceeds P03-15 | Check gain/load/P03-15 |
| 4 | Encoder alarm | Encoder connection fault | Check encoder wiring |
| 7 | Overload | Motor overloaded | Reduce load or change motor |

Green LED: Normal operation or offline/ready
Red LED: Alarm or protection active

---

## Complete Alarm Code Table

### Hardware Faults (AL.0xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.051 | EEPROM parameter anomaly | Wiring | Correct wiring, re-power. If persistent, replace drive. |
| AL.052 | Programmable logic config failure | - | Contact manufacturer |
| AL.053 | Initialization failed | MCU power | Re-power. If persistent, replace drive. |
| AL.054 | System exception | - | Contact manufacturer |
| AL.060 | Product model selection fault | - | Verify model settings |
| AL.061 | Product matching fault | - | Verify motor/driver match |
| AL.062 | Parameter storage failure | - | Re-power, reset parameters |
| AL.063 | Overcurrent detection | Short circuit in UVW or B1/B3 | Check motor wiring, check for shorts |
| AL.064 | Output to ground short | Ground fault in power stage | Check output wiring |
| AL.066 | Servo unit power supply low | Internal power fault | Check power supply |
| AL.070 | AD sampling fault 1 | Hardware | May need replacement |
| AL.071 | Current sampling fault | Current sensor | Check connections, may need replacement |

### Configuration Faults (AL.1xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.100 | Parameter combination anomaly | Invalid parameter combo | Review and correct parameters |
| AL.101 | AI setting fault | Analog input config | Check analog input parameters |
| AL.102 | DI distribution fault | Two inputs same function | Set unique DI functions |
| AL.103 | DO distribution fault | Two outputs same function | Set unique DO functions |
| AL.105 | Electronic gear setting error | P03-10/P03-11 invalid | Correct electronic gear ratio |
| AL.106 | Abnormal frequency division pulse | Output pulse config | Check pulse output parameters |
| AL.110 | Parameters should be reset | Changed params need re-power | Re-power the driver |
| AL.120 | Invalid servo command | Bad command received | Check controller output |

### Voltage/Power Faults (AL.4xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.401 | Under voltage | Supply < rated | Check power supply and wiring |
| AL.402 | Over voltage | Supply > rated | Use correct voltage source or stabilizer |

### Motor Protection (AL.4xx-AL.5xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.410 | Overload (instantaneous max) | Mechanical stuck/start | Check mechanical, reduce load |
| AL.411 | Driver overload | Continuous high current | Reduce duty cycle |
| AL.412 | Motor overload (continuous max) | d13.oL monitoring | Reduce load or upsize motor |
| AL.420 | Over speed | Speed > P04-05 | Adjust frequency, check P04-05 |
| AL.421 | Out of control check out | - | Check gains |
| AL.422 | Speed fault | - | Check speed command |
| AL.425 | AI sampling voltage too high | Analog input OV | Check analog input |
| AL.435 | Impulse current limits overload | Regen resistance | Check braking resistor |
| AL.436 | DB overload | Regen braking | Check P00-30 and braking resistor |
| AL.440 | Radiator overheating | Temp > 95C | Improve cooling. If persistent, send for service. |
| AL.441 | Motor overheat fault | Internal temp | Improve cooling |

### Position Faults (AL.5xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.500 | Freq division pulse overspeed | Pulse output too fast | Reduce pulse output frequency |
| AL.501 | Position error too big | Deviation > P03-15 | Increase P03-15, check gains, reduce load, check torque limits |
| AL.502 | Full closed loop deviation | Encoder/motor mismatch | Check full closed loop setup |
| AL.505 | P command input pulse exception | Pulse freq too high | Reduce input pulse frequency |
| AL.550 | Failure identification of inertia | Auto-tune failed | Re-run auto-tune under load |
| AL.551 | Return to origin timeout fault | Homing failed | Check homing setup |
| AL.552 | Angle identify failed | - | Check encoder/motor match |

### Encoder Faults (AL.6xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.600 | Encoder output power short | Encoder power fault | Check encoder wiring |
| AL.610 | Incremental decoder offline | Hall signal anomaly | Check encoder connections |
| AL.611 | Incremental encoder Z signal loss | Z pulse missing | Check Z signal wiring |
| AL.620 | Bus type encoder offline | - | Check bus encoder |
| AL.621 | Motor encoder EEPROM abnormal | - | May need replacement |
| AL.622 | Motor encoder EEPROM verify error | - | May need replacement |

### Warnings (AL.9xx)

| Code | Description | Check | Fix |
|------|-------------|-------|-----|
| AL.900 | Excessive position deviation | Position error warning | Check gains and mechanical |
| AL.901 | Position deviation ON too large | Deviation at servo ON | Check initial position |
| AL.910 | Motor overload (warning) | Load approaching limit | Reduce load |
| AL.912 | Driver overload (warning) | Driver approaching limit | Reduce load |
| AL.941 | Parameter changes need reconnection | - | Re-power |
| AL.942 | EEPROM frequent write warning | Too many writes | Reduce parameter change frequency |
| AL.943 | Serial communication exception | Baud rate or wiring | Check P00-21, add filter, reduce baud rate |
| AL.950 | Overpass warning | - | Check speed limits |
| AL.971 | Undervoltage warning | Voltage dropping | Check power supply |

---

## Clearing Alarms

1. Fix the underlying cause
2. Toggle ENA (enable) signal: ENA+ to 5V, then back
3. Or power cycle the servo
4. Some alarms require parameter correction before clearing
