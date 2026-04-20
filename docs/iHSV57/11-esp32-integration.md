# ESP32 Integration Notes

## Hardware Requirements

### RS232 Level Shifter

The iHSV57 serial port uses RS232 levels. The ESP32 uses 3.3V TTL. You need a
level shifter between them.

**Recommended**: MAX3232 breakout board (cheap, widely available, works on 3.3V VCC)

```
ESP32                    MAX3232                  iHSV57
GPIO TX -----> T1IN      T1OUT -----> RX (pin 2, brown/white)
GPIO RX <----- R1OUT     R1IN  <----- TX (pin 4, blue/white)
GND     <----> GND       GND   <----> GND (pin 3, blue)
3.3V    -----> VCC
```

**Alternative**: If the servo's RS232 port turns out to be TTL-level (some reports
suggest this), you could potentially connect directly. Test with a multimeter:
- If TX pin idles at ~3.3V or ~5V: TTL level, may work direct
- If TX pin idles at ~-5V to -12V: True RS232, needs MAX3232

### Available ESP32 UARTs

| UART | Default Pins | Typical Use |
|------|-------------|-------------|
| UART0 | GPIO 1 (TX), GPIO 3 (RX) | USB debug/serial console |
| UART1 | GPIO 10 (TX), GPIO 9 (RX) | Remappable - often conflicts with flash |
| UART2 | GPIO 16 (TX), GPIO 17 (RX) | **Best choice for servo Modbus** |

UART2 is recommended because UART0 is used for debug and UART1 pins often
conflict with the SPI flash on common ESP32 modules.

The ESP32 OSSM board currently uses these pins:
- GPIO 14: SERVO_PULSE (step)
- GPIO 27: SERVO_DIR
- GPIO 26: SERVO_ENABLE

UART2 pins (GPIO 16, 17) appear to be available, but verify against your
specific board layout.

### ESP32 Built-in RS485 Support

The ESP32 UART hardware actually has built-in RS485 support with automatic
DE/RE (driver enable) pin control. However, since the iHSV57 uses RS232 (not RS485),
this feature isn't directly useful. It WOULD be useful for the 57AIM motor which
uses RS485.

---

## Software: Modbus RTU Libraries for ESP32/PlatformIO

### eModbus (miq19/eModbus) - Used by iHSV57-Arduino

```ini
; platformio.ini
lib_deps = miq19/eModbus@^1.6.0
```

- Supports Modbus RTU master mode
- Works on ESP32 and ESP8266
- Handles CRC automatically
- Async operation with callbacks

```cpp
#include "ModbusClientRTU.h"

ModbusClientRTU modbus(Serial2);

void setup() {
    Serial2.begin(57600, SERIAL_8E1, 16, 17); // RX=16, TX=17
    modbus.begin();
}

// Read register
modbus.addRequest(token, 1, READ_HOLD_REGISTER, 0x0080, 1);

// Write register
modbus.addRequest(token, 1, WRITE_HOLD_REGISTER, 0x0006, 0x0000);
```

### ModbusMaster (4-20ma/ModbusMaster)

```ini
lib_deps = 4-20ma/ModbusMaster@^2.0.1
```

Simpler synchronous API, widely used:

```cpp
#include <ModbusMaster.h>

ModbusMaster node;
Serial2.begin(57600, SERIAL_8E1, 16, 17);
node.begin(1, Serial2); // slave address 1

// Read register
uint8_t result = node.readHoldingRegisters(0x0080, 1);
if (result == node.ku8MBSuccess) {
    uint16_t value = node.getResponseBuffer(0);
}

// Write register
result = node.writeSingleRegister(0x0006, 0x0000);
```

### Espressif esp-modbus

The official Espressif Modbus component. More complex but production-quality.
May be overkill for simple register reads/writes.

---

## Implementation Strategy for OSSM

### Phase 1: Monitoring (Safe, additive)

Add Modbus read capability alongside existing step/direction control:
1. Connect UART2 to servo RS232 via MAX3232
2. Initialize Modbus RTU master at 57600, 8E1
3. Verify connection: read register 0x0080, expect 0x0012
4. Periodically read monitoring registers:
   - Position feedback (d01.F.PU / d08.F.SP)
   - Motor speed (d08.F.SP)
   - Torque (d10.F.tQ)
   - Temperature (d24.Ath)
   - Alarm state
5. Report telemetry over BLE/WebSocket alongside existing data

This adds zero risk - step/direction still handles motion.

### Phase 2: Speed Control (Moderate complexity)

Replace step/direction with direct speed control:
1. Set P01-01 = 1 (speed mode) via Modbus or JMC software
2. Set P04-00 = 2 (digital speed via communication)
3. Set P06-40 = 300 (required for >1000 RPM)
4. Write desired speed to P04-02 (-6000 to 6000 RPM)
5. Read actual speed from monitoring register for feedback
6. Handle acceleration internally via P04-14/P04-15

Advantages:
- No pulse generation overhead
- Servo handles acceleration/deceleration smoothly
- Bidirectional with simple signed value
- Can read position/speed feedback simultaneously

### Phase 3: Position Control (Complex)

Direct position commands over Modbus:
1. Set P03-00 = 1 (communication position control)
2. Send target positions over Modbus
3. Use Ruckig or similar trajectory planner (like ossm-rs)
4. Read position feedback for closed-loop control from ESP32

This is what ossm-rs does with the 57AIM. May require discovering if the
iHSV57 supports a proprietary position command similar to the 57AIM's 0x7B.

---

## Memory/Resource Considerations

The ESP32 in the OSSM is already running:
- BLE (NimBLE) - significant RAM usage
- WiFi + WebSocket - significant RAM usage
- FastAccelStepper - timer interrupts
- StrokeEngine - motion planning
- FreeRTOS queues for BLE/Serial

Adding Modbus RTU requires:
- One additional UART (UART2)
- ~2-4 KB RAM for Modbus library
- Periodic task for polling (10-100 Hz)
- Not much CPU - Modbus is simple framing with CRC

This should fit comfortably. The main concern is ensuring the Modbus polling
doesn't interfere with the main loop timing, especially if step/direction
is still active (Phase 1).

---

## Regenerative Energy Warning

Rapid deceleration in OSSM creates voltage spikes from back-EMF. The iHSV57 has
overvoltage protection (AL.402), but repeated triggering is not ideal.

For high-speed applications:
- Consider an external regenerative braking resistor
- Set P04-15 (deceleration time) conservatively
- Monitor d23.dcp (DC bus voltage) during operation
- The servo's internal braking capability is limited
