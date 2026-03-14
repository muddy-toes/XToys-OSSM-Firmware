# JMC iHSV57 Servo Motor - Complete Reference

**Manufacturer**: Shenzhen Just Motion Control (JMC / 杰美康)
**Website**: https://www.jmc-motor.com
**Model**: iHSV57-30-18-36 (180W, 36V, NEMA 23)
**Firmware**: V6.xx (current production)

## Table of Contents

1. [Overview & Specifications](01-overview-and-specs.md)
2. [Communication Protocol (Modbus RTU)](02-modbus-protocol.md)
3. [Register Map](03-register-map.md)
4. [Control Modes](04-control-modes.md)
5. [DIP Switch Settings](05-dip-switches.md)
6. [Wiring & Connectors](06-wiring-and-connectors.md)
7. [PID Tuning](07-pid-tuning.md)
8. [Alarm Codes](08-alarm-codes.md)
9. [Monitoring Registers](09-monitoring-registers.md)
10. [Software & Tools](10-software-and-tools.md)
11. [ESP32 Integration Notes](11-esp32-integration.md)
12. [OSSM-Specific Notes](12-ossm-notes.md)
13. [Sources & References](13-sources.md)

## Quick Reference

| Parameter | Value |
|-----------|-------|
| Protocol | Modbus RTU over RS232 |
| Baud rate | 57600 (default, configurable) |
| Data format | 8 data bits, Even parity, 1 stop bit |
| Slave address | 1 (default, configurable 0-255) |
| Function codes | 0x03 (read), 0x06 (write single) |
| CRC | CRC-16/MODBUS (poly 0xA001, init 0xFFFF, little-endian) |
| Connection check | Read register 0x0080, expect 0x0012 |
| RS232 connector | 5-pin spring terminal: NC, RX, GND, TX, VCC |

## Key Insight

The iHSV57 can do FAR more than step/direction control. Over Modbus RTU you can:
- **Read real-time telemetry**: position, velocity, torque, temperature, voltage, load
- **Set speed directly**: P04-00=2, then write speed (-6000 to 6000 RPM) to P04-02
- **Set torque directly**: P05-10/P05-11 for torque limits (0-300%)
- **Digital position mode**: P03-00=1 for communication-controlled positioning
- **Tune PID parameters** on the fly
- **Monitor alarms and errors**
- **Change control modes** without power cycling (some require servo OFF state)

## Cloned Repositories

All reference repositories are in `docs/iHSV57-repos/`:

| Repository | Description |
|------------|-------------|
| `go-servoc/` | Go CLI tool with complete Modbus protocol implementation |
| `iHSV-Servo-Tool/` | Python GUI with V5/V6 register maps and live monitoring |
| `iHSV57-Arduino/` | Arduino/ESP32 Modbus library (minimal, proof of concept) |
| `JMC-Servo-Configuration/` | V1/V2 PDF manuals, JMC PC software, wiring guides |
| `JMC_servo_tuning/` | Python parameter extraction and tuning guide |
| `ossm-rs/` | Rust OSSM firmware with direct Modbus motor control |
