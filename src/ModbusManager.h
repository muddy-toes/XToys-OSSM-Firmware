#pragma once

#include <Arduino.h>

// ---- Modbus RTU over RS232 to iHSV57 servo ----
// The OSSM Reference Board has an SP3232E RS232 transceiver on UART2
// (GPIO16 RX, GPIO17 TX) with a 3-pin header (TX, RX, GND).
// Wire this to the servo's 5-pin RS232 connector:
//   Board TX -> Servo RX (pin 2, brown/white)
//   Board RX -> Servo TX (pin 4, blue/white)
//   Board GND -> Servo GND (pin 3, blue)

// RS232 / UART2 pins (directly to SP3232E on OSSM board)
#define MODBUS_RX_PIN     16
#define MODBUS_TX_PIN     17
#define MODBUS_BAUD       57600
#define MODBUS_SLAVE_ADDR 0x01

// ---- iHSV57 V6 Modbus Register Addresses ----

// Connection check (read returns 0x0012 on V5; may differ on V6)
#define REG_CONNECTION_CHECK  0x0080

// Control mode (P01-01): 0=position, 1=speed, 2=torque
#define REG_CONTROL_MODE      0x0065

// Rigidity (P01-03): 0-31, adjustable at runtime
#define REG_RIGIDITY          0x0067

// Gain switching (P02-30) - currently disabled, see session notes for history
#define REG_GAIN_SWITCH_MODE  0x00E6  // P02-30: 0=disabled, 3=torque, 9=speed

// Telemetry - monitoring registers (read-only, V6 addresses)
#define REG_MON_SPEED_CMD     0x0841  // Speed command (RPM, signed)
#define REG_MON_SPEED_FB      0x0842  // Motor actual speed (RPM, signed)
#define REG_MON_TORQUE_CMD    0x0843  // Torque command (%, signed)
#define REG_MON_TORQUE_FB     0x0844  // Torque feedback (%, signed)
// Position registers (available for future use):
// 0x0834/0x0835 = Position command (32-bit), 0x0836/0x0837 = Position feedback (32-bit)

// ---- Compliance tuning constants ----
// Written once to the servo after homing. Zero runtime writes.
// The servo's internal 2kHz loop handles gain switching automatically
// based on these fixed thresholds.

// Gain switching disabled (P02-30 = 0). We tested modes 3 (torque-based) and
// 9 (speed-based) with soft gains at 50% and 75% of firm. Results:
// - Mode 3: reversal artifacts (every direction change triggers switching)
// - Mode 9: clean reversals, but compliance imperceptible even at 50% gain
// - 50% gain + 3x integral: motor went limp, skipped belt teeth
// The servo's position loop corrects too fast for gain switching to create
// perceptible "give." Keeping Modbus for telemetry only.
#define COMPLIANCE_SWITCH_MODE    0

// ---- Telemetry data structure ----

struct ServoTelemetry {
    int16_t speedCommand;     // RPM
    int16_t speedFeedback;    // RPM
    int16_t torqueCommand;    // % of rated
    int16_t torqueFeedback;   // % of rated
    bool valid;               // true if data has been read at least once
};

// ---- Public API ----

namespace ModbusManager {
    // Call once from setup() - initializes UART2 but does NOT talk to servo yet
    void setup();

    // Call from main loop - handles non-blocking Modbus communication
    void loop();

    // Call after homing completes - writes gain switching config to servo
    // Returns true if servo responded and was configured successfully
    bool initServo();

    // Check if servo is connected and responding
    bool isConnected();

    // Get latest telemetry snapshot
    const ServoTelemetry& getTelemetry();
}
