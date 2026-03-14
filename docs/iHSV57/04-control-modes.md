# iHSV57 Control Modes

## P01-01: Control Mode Setting

| Value | Mode | Description |
|-------|------|-------------|
| 0 | Position control | Default. Step/direction, dual pulse, or digital position |
| 1 | Speed control | Analog, digital, PWM, or Modbus speed control |
| 2 | Torque control | Analog, digital, or PWM torque control |
| 3 | Speed/Torque switching | Switch between speed and torque via DI port |
| 4 | Position/Speed switching | Switch between position and speed via DI port |
| 5 | Position/Torque switching | Switch between position and torque via DI port |
| 6 | Full closed loop | Uses external encoder for full closed-loop control |

**P01-01 can only be changed when the driver is DISABLED (STOP state).**

---

## Position Control (P01-01 = 0)

### Sub-modes via P03-00 (Location Command Source)

| P03-00 | Mode | Description |
|--------|------|-------------|
| 0 | Pulse command | Standard step/direction mode (what OSSM uses) |
| 1 | Communication control | **Position commands via Modbus!** |

### Pulse/Direction Mode (P03-00 = 0, standard OSSM)

The ESP32 sends step pulses on PUL+/- and direction on DIR+/-. This is one-way
communication - the ESP32 has no feedback from the motor.

Relevant parameters:
- P03-03: Inverse command pulse direction (0=normal, 1=reverse)
- P03-04: Pulse filter (0=0.1us, 1=0.4us, 2=0.8us, 3=1.6us)
- P03-09: Pulses per revolution (0=use electronic gearing, else direct count)
- P03-10/P03-11: Electronic gear ratio (numerator/denominator)

### Digital Position Mode (P03-00 = 1)

Position commands come over Modbus serial. This is the mode that could replace
step/direction control.

Relevant parameters:
- P03-05: Position complete judgment condition
- P03-06: Position complete range (encoder units)
- P03-15: Position deviation too large alarm threshold
- P03-16: Position command smoothing filter (0-1000 ms)
- P03-45: Digital location instruction cache mode (0=immediate, 1=finish first)
- P03-46: Max RPM in digital position mode (0-6000)

**Note**: The exact Modbus register for sending position targets in this mode is not
fully documented in English. The iHSV-Servo-Tool and ossm-rs projects have the
most information. The 57AIM motor uses a proprietary function code 0x7B for
absolute position commands.

### V5 Control Mode Register Values (go-servoc)

On V5 firmware, the control mode is split across two registers (0x0006 and 0x0007):

| Mode | Name | Reg 0x0006 | Reg 0x0007 |
|------|------|------------|------------|
| Position: Pulse+Dir CCW | pos_pulse_ccw_dir | 0x0000 | 0x0000 |
| Position: Pulse+Dir CW | pos_pulse_cw_dir | 0x0000 | 0x0002 |
| Position: Dual Pulse CCW | pos_dipulse_ccw_dir | 0x0000 | 0x0001 |
| Position: Dual Pulse CW | pos_dipulse_cw_dir | 0x0000 | 0x0003 |
| Position: Digital | pos_digital | 0x0020 | 0x0000 |
| Position: Ramp Enable | pos_ramp_enable | 0x0060 | 0x0000 |
| Velocity: Analog CW | vel_analog_cw | 0x000C | 0x0000 |
| Velocity: Analog CCW | vel_analog_ccw | 0x000C | 0x0004 |
| Velocity: Digital PWM Half | vel_digital_pwm_half | 0x0004 | 0x0008 |
| Velocity: Digital PWM Inv | vel_digital_pwm_inv | 0x0004 | 0x0010 |
| Velocity: Ramp Enable | vel_ramp_enable | 0x0084 | 0x0010 |
| Torque: Analog CW | torque_analog_cw | 0x0003 | 0x0000 |
| Torque: Analog CCW | torque_analog_ccw | 0x0003 | 0x0004 |
| Torque: Digital PWM Half | torque_digital_pwm_half | 0x0001 | 0x0008 |
| Torque: Digital PWM Inv | torque_digital_pwm_inv | 0x0001 | 0x0010 |

---

## Speed Control (P01-01 = 1)

### Sub-modes via P04-00 (Speed Command Source)

| P04-00 | Source | Description |
|--------|--------|-------------|
| 0 | External analog | Speed from voltage on analog input |
| 1 | Digital (parameter) | Speed from P04-02 register |
| 2 | Digital (communication) | Speed from Modbus write |
| 3 | Internal / PWM | Multiple preset speeds or PWM on PUL+ |

### Modbus Speed Control (P04-00 = 1 or 2)

Set P01-01 = 1, P04-00 = 1 (or 2), then write speed to **P04-02**:

- **Register**: P04-02 (address varies by firmware version)
- **Range**: -6000 to 6000 RPM (negative = reverse)
- **Effective**: Immediately while running

Relevant parameters:
- P04-06: Forward speed limit (0-6000 RPM, default 5000)
- P04-07: Reverse speed limit (0-6000 RPM, default -5000)
- P04-14: Acceleration time (0-10000, unit: 1ms/1000rpm, default 500)
- P04-15: Deceleration time (0-10000, unit: 1ms/1000rpm, default 500)
- P04-05: Overspeed alarm value (0-6500 RPM, default 6400)

### PWM Speed Control (P04-00 = 3)

PUL+ accepts 5V PWM, 1-20 kHz:
- 10-90% duty cycle controls speed proportionally
- DIR+ sets direction
- Speed range set by P04-06/P04-07

### Hidden Parameter: P06-40

**Set P06-40 = 300 for speeds above 1000 RPM.**

This parameter is only visible in JMC software when connecting as "AC servo motor"
instead of "Integrated servo motor". Without this setting, the motor will be limited
to ~1000 RPM regardless of P04-02 value. Source: Rocketronics guide.

---

## Torque Control (P01-01 = 2)

### Sub-modes via P05-00 (Torque Command Source)

| P05-00 | Source | Description |
|--------|--------|-------------|
| 0 | Analog | Torque from analog voltage input |
| 1 | Digital | Torque from parameter setting |
| 2 | Analog + speed limit | Analog torque with speed limiter |
| 3 | Digital + speed limit | Digital torque with speed limiter |

### Torque Parameters

- **P05-10**: Internal positive torque limit (0-300%, where 100=rated, 300=3x rated)
- **P05-11**: Internal reverse torque limit (0-300%)
- **P05-02**: Speed limiter in torque mode (0-6000 RPM)

### V6 Torque Register

The iHSV57-Arduino library writes torque to register **0x01FE**. This maps to
the internal torque command.

### Safety Note

Start with low torque values (P05-10 = 10, which is 10% rated) for testing.
At 10% the motor can be stopped by hand. Full torque (200-300%) can be dangerous.

---

## Key Takeaway for OSSM

The most promising mode for upgrading OSSM beyond step/direction is **speed control
via Modbus** (P01-01=1, P04-00=2), because:

1. You can set speed directly without generating step pulses
2. The servo handles acceleration/deceleration internally (P04-14/P04-15)
3. You get bidirectional control (-6000 to +6000 RPM)
4. You can simultaneously read position/velocity feedback for closed-loop control
5. Speed commands are effective immediately - no waiting for pulse generation

The theoretical update rate at 57600 baud is ~200-300 Hz, which is more than adequate
for smooth motion when the servo is handling its own acceleration profiles.

Digital position mode (P03-00=1) is also interesting but less well documented for
the iHSV57 specifically. The 57AIM servo in ossm-rs uses a proprietary command
(function code 0x7B) for position, which the iHSV57 may or may not support.
