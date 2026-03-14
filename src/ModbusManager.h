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

// Gain switching config (P02-xx)
#define REG_GAIN_SWITCH_MODE  0x00E6  // P02-30: 0-10, mode selector
#define REG_GAIN_SWITCH_LEVEL 0x00E7  // P02-31: 0-20000, threshold
#define REG_GAIN_SWITCH_HYST  0x00E8  // P02-32: 0-20000, hysteresis
#define REG_GAIN_SWITCH_DELAY 0x00E9  // P02-33: switching delay (0.1ms units)

// PID gains - set 1 (firm) and set 2 (soft)
#define REG_POS_GAIN_1        0x00C8  // P02-00: position gain 1 (0-3000, 0.1/S units)
#define REG_POS_GAIN_2        0x00C9  // P02-01: position gain 2
#define REG_SPD_GAIN_1        0x00D2  // P02-10: speed proportional gain 1 (0.1 Hz units)
#define REG_SPD_GAIN_2        0x00D5  // P02-13: speed proportional gain 2
#define REG_SPD_INTEGRAL_1    0x00D3  // P02-11: speed integral constant 1 (0.1 ms units)
#define REG_SPD_INTEGRAL_2    0x00D6  // P02-14: speed integral constant 2

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

// Gain switching mode 3 = switch to soft gains when torque > threshold
#define COMPLIANCE_SWITCH_MODE    3

// Switching threshold (P02-31 units)
// 1000 bits = 25% rated torque (from manual: "Torque unit: 1000 bit = 25% rated torque")
// Normal endpoint reversals peak ~400 in torque feedback. We set threshold above
// that so compliance only triggers on actual external resistance, not normal stroking.
#define COMPLIANCE_DEFAULT_LEVEL  2000  // ~50% rated torque - above normal reversal peaks
#define COMPLIANCE_HYSTERESIS     200   // P02-32: prevents chattering between gain sets
#define COMPLIANCE_SWITCH_DELAY   100   // P02-33: 10ms delay before switching back (0.1ms units)

// Gain values: firm (gain set 1) and soft (gain set 2)
// Firm gains are read from servo at startup (preserving auto-tune values).
// Soft gains are slightly reduced - just enough to feel compliant without
// losing position authority. The motor must never go limp or drop.
// First test with 50% reduction caused belt skip. These are ~25% reduction.
#define FIRM_POS_GAIN   480   // P02-00: 48.0 1/S (factory default, overridden by servo's actual value)
#define SOFT_POS_GAIN   360   // P02-01: 36.0 1/S (~75% of factory firm)
#define FIRM_SPD_GAIN   270   // P02-10: 27.0 Hz (factory default, overridden by servo's actual value)
#define SOFT_SPD_GAIN   200   // P02-13: 20.0 Hz (~75% of factory firm)
#define FIRM_SPD_INT    100   // P02-11: 10.0 ms (factory default)
#define SOFT_SPD_INT    150   // P02-14: 15.0 ms (only slightly slower than firm)

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
