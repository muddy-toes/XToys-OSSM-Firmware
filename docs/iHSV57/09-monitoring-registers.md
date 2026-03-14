# Monitoring Registers

## V6 Firmware - Modbus Monitoring Registers

These registers can be read in real-time while the servo is running.
Use function code 0x03 (Read Holding Register).

### Position Data (32-bit, 2 registers each)

| Address | Display Code | Description | Unit |
|---------|-------------|-------------|------|
| 0x0834 (hi), 0x0835 (lo) | d00.C.PU | Position command total | instruction unit |
| 0x0836 (hi), 0x0837 (lo) | d01.F.PU | Position feedback total | instruction unit |
| 0x0838 (hi), 0x0839 (lo) | d02.E.PU | Position deviation | instruction unit |

For 32-bit values, read both registers with a single request (count=2) and combine:
```
value = (reg_hi << 16) | reg_lo
```

### Velocity Data (16-bit, signed)

| Address | Display Code | Description | Unit |
|---------|-------------|-------------|------|
| 0x0841 | d07.C.SP | Speed control instruction | RPM |
| 0x0842 | d08.F.SP | Motor actual speed | RPM |

### Torque Data (16-bit, signed)

| Address | Display Code | Description | Unit |
|---------|-------------|-------------|------|
| 0x0843 | d09.C.tQ | Torque command | % (100=rated) |
| 0x0844 | d10.F.tQ | Torque feedback | % (100=rated) |

### Additional Monitor Items (from display panel, register addresses TBD)

These are readable via the JMC software and iHSV-Servo-Tool. Register addresses
for V6 are in `iHSV_Properties.py`.

| Display Code | Description | Unit |
|-------------|-------------|------|
| d03.C.PE | Command pulses / encoder feedback | encoder unit |
| d04.F.PE | Position feedback pulse sum | encoder unit |
| d05.E.PE | Position deviation pulse number | encoder unit |
| d06.C.Fr | Pulse command input frequency | KPPS |
| d11.AG.L | Average torque (last 10 seconds) | % |
| d12.PE.L | Peak torque since power on | % |
| d13.oL | Overload rate (last 10 seconds) | % |
| d14.rG | Regenerative load factor | % |
| d16.I.Io | Digital input IO state | binary |
| d17.o.Io | Digital output IO state | binary |
| d18.AnG | Machine (shaft) angle | 0.1 degree |
| d19.HAL | Motor UVW phase sequence | - |
| d20.ASS | Absolute encoder single coil value | 0-0xFFFF |
| d21.ASH | Absolute encoder multi-turn value | - |
| d22.J-L | Real-time inertia ratio | % |
| d23.dcp | Main circuit DC voltage | V |
| d24.Ath | Driver temperature | degrees C |
| d25.tiE | Cumulative running time | seconds |
| d26.1.Fr | Resonance frequency 1 | Hz |
| d28.2.Fr | Resonance frequency 2 | Hz |
| d30.Ai1 | Analog input 1 voltage (V_REF) | 0.01V |
| d31.Ai2 | Analog input 2 voltage (T_REF) | 0.01V |

---

## V5 Firmware - Monitoring Registers

| Address(es) | Description | Signed | Size |
|-------------|-------------|--------|------|
| 0x0085, 0x0086 | Position command | No | 32-bit |
| 0x0087, 0x0088 | Real position | No | 32-bit |
| 0x0089 | Position error | Yes | 16-bit |
| 0x0090 | Velocity command | Yes | 16-bit RPM |
| 0x0091 | Real velocity | Yes | 16-bit RPM |
| 0x0092 | Velocity error | Yes | 16-bit RPM |
| 0x00A0 | Torque current command | Yes | 16-bit |
| 0x00A1 | Real torque current | Yes | 16-bit |

---

## Efficient Monitoring

The iHSV-Servo-Tool achieves ~100 Hz monitoring rate by reading multiple registers
in a single Modbus request. For example, reading all velocity+torque data:

```
TX: 01 03 08 41 00 04 [CRC]   // Read 4 registers starting at 0x0841
```

Returns d07.C.SP, d08.F.SP, d09.C.tQ, d10.F.tQ in one response.

At 57600 baud:
- 8-byte request: ~1.5 ms
- 13-byte response (4 registers): ~2.5 ms
- Total round-trip: ~4 ms + processing
- Achievable: ~200 reads/second for 4-register blocks

### Recommended Polling Strategy

For OSSM integration:
- **High priority (50-100 Hz)**: Speed, position (for motion feedback)
- **Medium priority (10 Hz)**: Torque, temperature
- **Low priority (1 Hz)**: Voltage, alarm state, running time
- **On-demand**: PID parameters, configuration reads
