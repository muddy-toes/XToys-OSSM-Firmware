# OSSM-Specific Notes

## Current Status

The iHSV57 is the **legacy motor** in the OSSM ecosystem. Research and Desire
has deprecated it in favor of the **57AIM30** ("Gold Motor"), which is 20% cheaper,
60% smaller, 50% lighter, and delivers 30% more peak force.

However, many OSSM builds still use the iHSV57, and understanding its serial
capabilities opens up significant firmware improvement possibilities.

## Standard OSSM Configuration

| Parameter | Setting |
|-----------|---------|
| Steps per revolution | 800 (DIP: OFF-ON-ON-ON, or software via P03-09) |
| Control mode | Position (step/direction) |
| Communication | One-way (ESP32 sends pulses, no feedback) |

## OSSM Firmwares and Motor Control

| Firmware | Motor Interface | Notes |
|----------|----------------|-------|
| KinkyMakers/OSSM-controller | Step/Dir (legacy) | Serial packets |
| XToys-OSSM-Firmware (ours) | Step/Dir via StrokeEngine | BLE/WebSocket/Serial JSON |
| OSSM-Sauce | Step/Dir via FastAccelStepper | WebSocket binary |
| LUST-motion | Step/Dir + IHSV_SERVO_V6 driver | REST API / MDNS |
| **ossm-rs** | **RS485 Modbus RTU (direct)** | BLE / ESP-NOW, 57AIM only |

**ossm-rs is the only firmware that has moved beyond step/dir** to direct servo
communication. It targets the 57AIM (not iHSV57) but the approach is transferable.

## Key Community Members

- **theelims**: Author of StrokeEngine (the motion library we use). Also maintains
  a fork of iHSV-Servo-Tool, suggesting deep knowledge of both motion control
  and servo serial communication.

- **MikesMachines**: Created the JMC-Servo-Configuration documentation repo.
  Was actively trying ESP32 Modbus control but got stuck on "Modbus overhead."

- **robert-budde**: Created iHSV-Servo-Tool, the best open-source tool for
  interacting with the iHSV57 over serial.

## 3.3V Signal Compatibility

OSSM GitHub issue #3 documented that step loss was caused by **3.3V incompatibility**.
The servo's opto-isolated inputs nominally expect 5V. While they technically trigger
at 3.5V, running at 3.3V is marginal and can cause unreliable pulse detection,
especially at high frequencies.

For the control signal inputs (PUL/DIR/ENA), consider:
- Level shifter to 5V
- Or common-anode wiring with external 5V pull-up

For the RS232 serial port, a MAX3232 handles the level shifting.

## MikesMachines' Observations on Servo Tuning

From the JMC-Servo-Configuration README:

**Rigidity (P01-02 / P01-03)**: "Has a huge impact on the feel of the machine.
Creates a springy feel on lower settings with slip and give. Too low and the
stroke doesn't have enough force to move. High settings stiffen up until it
starts vibrating. Can be changed while running. Potential for changing with
stroke direction - going in loose might be safe or more natural."

This suggests dynamic rigidity tuning over Modbus could be a unique feature:
- Lower rigidity during insertion (safer, more natural)
- Higher rigidity during withdrawal (more force, faster)
- Adjustable via the XToys interface

## 57AIM30 vs iHSV57 Comparison

| Feature | iHSV57-30-18-36 | 57AIM30 |
|---------|-----------------|---------|
| Power | 180W | 180W+ |
| Size | 150mm length | ~90mm length |
| Weight | 1.6 kg | ~0.8 kg |
| Serial interface | RS232 | RS485 |
| Protocol | Modbus RTU | Modbus RTU |
| Step/Dir | Yes | Yes |
| Position command over serial | Yes (P03-00=1) | Yes (func 0x7B) |
| Speed command over serial | Yes (P04-00=2) | Yes |
| Encoder resolution | 1000 CPR (4000 quad) | 32768 steps/rev |
| Auto-tune | V6 only (P01-03) | Yes |
| Price | ~$80-120 | ~$65-95 |
| OSSM status | Legacy | Current recommended |

## Potential Improvements via Modbus

1. **Real-time telemetry**: Position, speed, torque, temperature over BLE/WebSocket
2. **Dynamic rigidity**: Change P01-03 on the fly based on stroke direction
3. **Speed control mode**: Eliminate step pulse generation overhead
4. **Alarm monitoring**: Detect and report servo errors before they cause problems
5. **Over-temperature protection**: Read d24.Ath and reduce speed before overheat
6. **Position feedback**: Know actual position for true closed-loop control
7. **Torque limiting**: Dynamic torque limits for safety
8. **Diagnostic data**: Voltage, load, encoder status for maintenance alerts
