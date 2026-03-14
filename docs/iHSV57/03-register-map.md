# iHSV57 Modbus Register Map

## IMPORTANT: V5 vs V6 Firmware

The V5 and V6 firmwares use **completely different register addresses** for the same
parameters. The V5 map was reverse-engineered by go-servoc. The V6 map is documented
in the official manual and iHSV-Servo-Tool.

**Check your firmware version before using these addresses.**

---

## V5 Firmware Register Map

Source: go-servoc, iHSV-Servo-Tool

### Configuration Registers (Read/Write)

| Address | Parameter | Name | Range | Default | Unit |
|---------|-----------|------|-------|---------|------|
| 0x0006 | P00-01 | Control Mode 1/2 | see control modes | - | - |
| 0x0007 | P00-02 | Control Mode 2/2 | see control modes | - | - |
| 0x0008 | P00-03 | Mode2 bitfield | bit0=ext/int, bit1=pos_filter, bit2=vel_filter | - | - |
| 0x000A | P00-04 | Encoder line count | 0-65535 | 1000 | lines |
| 0x0031 | P00-05 | Input offset | - | - | - |
| 0x0032 | P00-06 | Weight coefficient | - | - | - |
| 0x003A | P00-27 | Temperature limit | - | - | - |
| 0x003B | P00-28 | Over voltage limit | - | - | V |
| 0x003C | P00-29 | Under voltage limit | - | - | V |
| 0x003D | P00-30 | I2T limit | - | - | - |
| 0x0040 | P00-09 | Position proportional (Pp) | 0-3000 | - | 1/S |
| 0x0041 | P00-10 | Position derivative (Pd) | 0-3000 | - | 1/S |
| 0x0042 | P00-11 | Position feedforward (Pff) | 0-100 | - | % |
| 0x0045 | P00-12 | Position filter | 0-7 | - | - |
| 0x0046 | P00-07 | E-Gear numerator | 1-65535 | 4000 | - |
| 0x0047 | P00-08 | E-Gear denominator | 1-65535 | 4000 | - |
| 0x0048 | P00-13 | Position error limit | 0-65535 | - | pulses |
| 0x0050 | P00-14 | Velocity proportional (Vp) | 1-2000 | - | Hz |
| 0x0051 | P00-15 | Velocity integral (Vi) | 1-2000 | - | Hz |
| 0x0052 | P00-16 | Velocity derivative (Vd) | - | - | - |
| 0x0053 | P00-17 | Accel feedforward (Aff) | - | - | - |
| 0x0054 | P00-18 | Velocity filter | 0-7 | - | - |
| 0x0055 | P00-19 | Continuous velocity | - | - | - |
| 0x0056 | P00-20 | Velocity limit | 0-6000 | - | RPM |
| 0x0057 | P00-21 | Acceleration | 0-255 | - | - |
| 0x0058 | P00-22 | Deceleration | 0-255 | - | - |
| 0x0060 | P00-23 | Current proportional (Cp) | - | - | - |
| 0x0061 | P00-24 | Current integral (Ci) | - | - | - |
| 0x0062 | P00-25 | Continuous current | - | - | - |
| 0x0063 | P00-26 | Peak current limit | - | - | - |
| 0x0080 | - | Connection check (read-only) | - | 0x0012 | - |

### V5 Monitoring Registers (Read-Only)

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

## V6 Firmware Register Map

Source: Official V2 manual, iHSV-Servo-Tool

### P00-xx Motor and Driver Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P00-00 | Motor number | 0-65535 | - | - | Stop | Re-power |
| P00-01 | Motor rated speed | 1-6000 | - | RPM | Stop | Re-power |
| P00-02 | Motor rated torque | 0.01-655.35 | - | N.M | Stop | Re-power |
| P00-03 | Rated current | 0.01-655.35 | - | A | Stop | Re-power |
| P00-04 | Motor moment of inertia | 0.01-655.35 | - | kg.cm2 | Stop | Re-power |
| P00-05 | Pole number | 1-31 | - | poles | Stop | Re-power |
| P00-10 | Encoder lines | 0-65535 | - | - | Stop | Re-power |
| P00-11 | Encoder Z pulse angle | 0-65535 | - | - | Stop | Re-power |
| P00-12..17 | Initial rotor angles 1-6 | 0-360 | - | degree | Stop | Re-power |
| P00-21 | RS232 baud rate | 0-3 | 2 | - | Stop | Re-power |
| P00-23 | Slave address | 0-255 | 1 | - | Stop | Re-power |
| P00-24 | Modbus baud rate | 0-7 | 7 | - | Stop | Re-power |
| P00-25 | Parity (check way) | 0-3 | 1 | - | Stop | Re-power |
| P00-26 | Modbus response delay | 0-100 | 0 | ms | Stop | Re-power |
| P00-42 | Overvoltage threshold | 0-300 | 0 | V | Stop | Re-power |

### P01-xx Main Control Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P01-01 | Control mode | 0-6 | 0 | - | Stop | Immediate |
| P01-02 | Auto adjustment mode | 0-2 | - | - | Run | Immediate |
| P01-03 | Auto rigidity setting | 0-31 | 13 | - | Run | Immediate |
| P01-04 | Moment of inertia ratio | 0-100 | 1 | times | Run | Immediate |
| P01-30 | Brake command delay | 0-255 | 100 | ms | Run | Immediate |
| P01-31 | Speed limit for brake | 0-3000 | - | RPM | Run | Immediate |
| P01-32 | Servo OFF lock wait time | 0-255 | 100 | ms | Run | Immediate |

### P02-xx Gain Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P02-00 | Position control gain 1 | 0-3000 | 48.0 | 1/S | Run | Immediate |
| P02-01 | Position control gain 2 | 0-3000 | 57.0 | 1/S | Run | Immediate |
| P02-03 | Speed feedforward gain | 0-100 | 30.0 | % | Run | Immediate |
| P02-04 | Speed FF smoothing constant | 0-64.00 | 0.5 | ms | Run | Immediate |
| P02-10 | Speed proportional gain 1 | 1-2000 | 27.0 | Hz | Run | Immediate |
| P02-11 | Speed integral constant 1 | 0.1-1000 | 10.0 | ms | Run | Immediate |
| P02-12 | Pseudo diff FF control 1 | 0-100 | 100.0 | % | Run | Immediate |
| P02-13 | Speed proportional gain 2 | 1-2000 | 27.0 | Hz | Run | Immediate |
| P02-14 | Speed integral constant 2 | 0.1-1000 | 1000.0 | ms | Run | Immediate |
| P02-15 | Pseudo diff FF control 2 | 0-100 | 100.0 | % | Run | Immediate |
| P02-19 | Torque feedforward gain | 0-30000 | 0 | % | Run | Immediate |
| P02-20 | Torque FF smoothing constant | 0-64.00 | 0.8 | ms | Run | Immediate |
| P02-30 | Gain switching mode | 0-10 | 0 | - | Run | Immediate |
| P02-31 | Gain switching level | 0-20000 | 800 | - | Run | Immediate |
| P02-32 | Gain switching hysteresis | 0-20000 | 100 | - | Run | Immediate |
| P02-33 | Gain switching delay | 0-1000 | 10.0 | ms | Run | Immediate |
| P02-34 | Position gain switching time | 0-1000 | 10.0 | ms | Run | Immediate |
| P02-41 | Mode switch level | 0-20000 | 10000 | - | Run | Immediate |
| P02-50 | Torque instruction added | -100-100 | 0 | % | Run | Immediate |
| P02-51 | Forward torque compensation | -100-100 | 0 | % | Run | Immediate |
| P02-52 | Reverse torque compensation | -100-100 | 0 | % | Run | Immediate |

### P03-xx Position Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P03-00 | Location command source | 0-1 | 0 | - | Stop | Immediate |
| P03-03 | Inverse command pulse | 0-1 | 0 | - | Stop | Immediate |
| P03-04 | Location pulse filter | 0-3 | 2 | - | Run | Immediate |
| P03-05 | Location complete judgment | 0-2 | 1 | - | Run | Immediate |
| P03-06 | Location complete range | 0-65535 | 30 | encoder | Run | Immediate |
| P03-09 | Pulses per revolution | 0-65535 | 4000 | pulses | Run | Re-power |
| P03-10 | Electronic gear numerator | 1-65535 | 4000 | - | Run | Re-power |
| P03-11 | Electronic gear denominator | 1-65535 | 4000 | - | Run | Re-power |
| P03-15 | Position deviation too large | 0-65535 | 0 | inst*10 | Run | Immediate |
| P03-16 | Position smoothing filter | 0-1000 | 0 | ms | Run | Immediate |

**P03-00 values:**
- **0**: Pulse command (step/direction - standard OSSM mode)
- **1**: Numbers given, used for communication control (Modbus position mode!)

### P04-xx Speed Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P04-00 | Speed command source | 0-3 | 1 | - | Stop | Immediate |
| P04-02 | Digital speed given value | -6000 to 6000 | 0 | RPM | Run | Immediate |
| P04-05 | Overspeed alarm value | 0-6500 | 6400 | RPM | Run | Immediate |
| P04-06 | Forward speed limit | 0-6000 | 5000 | RPM | Run | Immediate |
| P04-07 | Reverse speed limit | 0-6000 | -5000 | RPM | Run | Immediate |
| P04-10 | Zero speed detection value | 0-200 | 40 | RPM | Run | Immediate |
| P04-14 | Acceleration time | 0-10000 | 500 | 1ms/1000rpm | Run | Immediate |
| P04-15 | Deceleration time | 0-10000 | 500 | 1ms/1000rpm | Run | Immediate |

**P04-00 values:**
- **0**: External analog instruction (voltage input)
- **1**: Digital instruction (parameter setting) - set speed via P04-02
- **2**: Digital instruction (communication) - set speed via Modbus
- **3**: Internal multiple sets of instructions / PWM

### P05-xx Torque Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P05-10 | Internal positive torque limit | 0-300 | 200.0 | % | Run | Immediate |
| P05-11 | Internal torque limit value | 0-300 | 200.0 | % | Run | Immediate |

100 = rated torque, 300 = 3x rated torque.

### P06-xx I/O Parameters

| Parameter | Name | Range | Default | Setting | Effective |
|-----------|------|-------|---------|---------|-----------|
| P06-00 | Enable input port effective level | 0-4 | 1 | Run | Re-power |
| P06-20 | Alarm output port effective level | 0-1 | 1 | Run | Re-power |
| P06-22 | Output port in place valid level | 0/1 | 1 | Run | Re-power |

### P08-xx Advanced Function Parameters

| Parameter | Name | Range | Default | Unit | Setting | Effective |
|-----------|------|-------|---------|------|---------|-----------|
| P08-19 | Feedback speed low-pass filter | 0-25.00 | 0.8 | ms | Run | Immediate |
| P08-20 | Torque command filter constant | 0-25.00 | 0.84 | ms | Run | Immediate |
| P08-25 | Disturbance torque compensation gain | 0-100 | 0 | % | Run | Immediate |
| P08-26 | Disturbance torque filter constant | 0-25.00 | 0.8 | ms | Run | Immediate |

### V6 Monitoring Registers (Read-Only)

| Address(es) | Display Code | Description | Unit |
|-------------|-------------|-------------|------|
| 0x0834, 0x0835 | d00.C.PU | Position command | 32-bit |
| 0x0836, 0x0837 | d01.F.PU | Position feedback | 32-bit |
| 0x0838, 0x0839 | d02.E.PU | Position deviation | 32-bit |
| - | d03.C.PE | Command pulses / feedback pulses | encoder unit |
| - | d04.F.PE | Position feedback pulse sum | encoder unit |
| - | d05.E.PE | Position deviation pulse number | encoder unit |
| - | d06.C.Fr | Pulse command input frequency | KPPS |
| 0x0841 | d07.C.SP | Speed control instruction | RPM |
| 0x0842 | d08.F.SP | Motor speed (actual) | RPM |
| 0x0843 | d09.C.tQ | Torque command | % |
| 0x0844 | d10.F.tQ | Torque feedback | % |
| - | d11.AG.L | Average torque (10s) | % |
| - | d12.PE.L | Peak torque since power on | % |
| - | d13.oL | Overload rate (10s) | % |
| - | d14.rG | Regenerative load factor | % |
| - | d16.I.Io | Input IO state | binary |
| - | d17.o.Io | Output IO state | binary |
| - | d18.AnG | Machine angle | 0.1 degree |
| - | d19.HAL | Motor UVW phase sequence | - |
| - | d20.ASS | Absolute encoder single coil | 0-0xFFFF |
| - | d21.ASH | Absolute encoder multi-turn | - |
| - | d22.J-L | Inertia ratio | % |
| - | d23.dcp | Main circuit voltage (DC) | V |
| - | d24.Ath | Driver temperature | degrees C |
| - | d25.tiE | Cumulative running time | seconds |
| - | d26.1.Fr | Resonance frequency 1 | Hz |
| - | d28.2.Fr | Resonance frequency 2 | Hz |
| - | d30.Ai1 | Analog input 1 voltage (V_REF) | 0.01V |
| - | d31.Ai2 | Analog input 2 voltage (T_REF) | 0.01V |

### V6 Torque Command Register

| Address | Description |
|---------|-------------|
| 0x01FE | Torque command value (from iHSV57-Arduino) |

---

## Parameter-to-Address Mapping Note

The official V6 manual lists parameters by code (P00-xx, P01-xx, etc.) but does NOT
explicitly list the Modbus register addresses for each parameter. The register addresses
were reverse-engineered by the iHSV-Servo-Tool project (`iHSV_Properties.py`, 9450 lines)
and the go-servoc project.

The full V6 address mapping is in:
`docs/iHSV57-repos/iHSV-Servo-Tool/iHSV_Properties.py`

This file contains the definitive register address for every parameter on both V5 and V6.
