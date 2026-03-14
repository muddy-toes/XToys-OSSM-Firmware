#include "ModbusManager.h"

// CRC-16/MODBUS lookup table derived from go-servoc by Torsten Curdt
// (https://github.com/tcurdt/go-servoc), licensed Apache 2.0.
// The table values are the standard CRC-16/MODBUS polynomial 0xA001.
static const uint16_t crcTable[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

static uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t n = data[i] ^ (uint8_t)crc;
        crc >>= 8;
        crc ^= crcTable[n];
    }
    return crc;
}

// ---- Low-level Modbus RTU framing ----

static HardwareSerial& modbusSerial = Serial2;

// Send a Modbus RTU frame (adds CRC automatically)
static void sendFrame(const uint8_t* data, size_t len) {
    uint16_t crc = crc16(data, len);
    modbusSerial.write(data, len);
    modbusSerial.write(crc & 0xFF);        // CRC low byte first
    modbusSerial.write((crc >> 8) & 0xFF); // CRC high byte
    modbusSerial.flush();  // Wait for TX to complete
}

// Read a response frame with timeout. Returns number of bytes read, 0 on timeout.
// Uses a longer initial timeout (waiting for first byte) and a shorter inter-character
// timeout (waiting for subsequent bytes within a frame).
static size_t readResponse(uint8_t* buf, size_t maxLen, uint32_t initialTimeoutMs = 200) {
    uint32_t start = millis();
    uint32_t timeout = initialTimeoutMs;
    size_t pos = 0;

    while (millis() - start < timeout && pos < maxLen) {
        if (modbusSerial.available()) {
            buf[pos++] = modbusSerial.read();
            // After first byte, switch to short inter-character timeout
            start = millis();
            timeout = 10;
        }
    }
    return pos;
}

// Verify CRC of a received frame
static bool verifyCrc(const uint8_t* data, size_t len) {
    if (len < 3) return false;
    uint16_t received = data[len - 2] | (data[len - 1] << 8);
    uint16_t calculated = crc16(data, len - 2);
    return received == calculated;
}

// ---- Modbus read/write operations (blocking, with timeout) ----

// Read a single 16-bit holding register. Returns true on success.
static bool readRegister(uint16_t regAddr, uint16_t* value) {
    uint8_t req[6] = {
        MODBUS_SLAVE_ADDR,
        0x03,                       // Function code: read holding register
        (uint8_t)(regAddr >> 8),    // Register address high
        (uint8_t)(regAddr & 0xFF),  // Register address low
        0x00, 0x01                  // Count = 1 register
    };

    // Flush any stale data
    while (modbusSerial.available()) modbusSerial.read();

    sendFrame(req, sizeof(req));

    uint8_t resp[32];
    size_t len = readResponse(resp, sizeof(resp));

    // Expected response: [addr] [0x03] [byte_count=2] [val_hi] [val_lo] [crc_lo] [crc_hi]
    if (len < 5) return false;
    if (resp[0] != MODBUS_SLAVE_ADDR) return false;
    // Check for Modbus error response (function code has bit 7 set)
    if (resp[1] == 0x83 && len >= 5) {
        Serial.printf("[MODBUS] Error response for read 0x%04X: exception code 0x%02X\n",
                      regAddr, resp[2]);
        return false;
    }
    if (len < 7) return false;
    if (resp[1] != 0x03) return false;
    if (resp[2] != 0x02) return false;
    if (!verifyCrc(resp, 7)) return false;

    *value = (resp[3] << 8) | resp[4];
    return true;
}

// Read multiple consecutive 16-bit holding registers. Returns true on success.
static bool readRegisters(uint16_t regAddr, uint16_t count, uint16_t* values) {
    uint8_t req[6] = {
        MODBUS_SLAVE_ADDR,
        0x03,
        (uint8_t)(regAddr >> 8),
        (uint8_t)(regAddr & 0xFF),
        (uint8_t)(count >> 8),
        (uint8_t)(count & 0xFF)
    };

    while (modbusSerial.available()) modbusSerial.read();
    sendFrame(req, sizeof(req));

    uint8_t resp[64];
    size_t expectedLen = 3 + count * 2 + 2; // addr + fc + bytecount + data + crc
    if (expectedLen > sizeof(resp)) return false;  // prevent buffer overflow
    size_t len = readResponse(resp, sizeof(resp));

    if (len < expectedLen) return false;
    if (resp[0] != MODBUS_SLAVE_ADDR) return false;
    if (resp[1] != 0x03) return false;
    if (resp[2] != count * 2) return false;
    if (!verifyCrc(resp, expectedLen)) return false;

    for (uint16_t i = 0; i < count; i++) {
        values[i] = (resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1];
    }
    return true;
}

// Write a single 16-bit holding register. Returns true on success (echo verified).
static bool writeRegister(uint16_t regAddr, uint16_t value) {
    uint8_t req[6] = {
        MODBUS_SLAVE_ADDR,
        0x06,                       // Function code: write single register
        (uint8_t)(regAddr >> 8),
        (uint8_t)(regAddr & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    while (modbusSerial.available()) modbusSerial.read();
    sendFrame(req, sizeof(req));

    // Response should echo the request exactly
    uint8_t resp[32];
    size_t len = readResponse(resp, sizeof(resp));

    if (len < 5) return false;
    if (resp[0] != MODBUS_SLAVE_ADDR) return false;
    // Check for Modbus error response
    if (resp[1] == 0x86 && len >= 5) {
        Serial.printf("[MODBUS] Error response for write 0x%04X: exception code 0x%02X\n",
                      regAddr, resp[2]);
        return false;
    }
    if (len < 8) return false;
    if (resp[1] != 0x06) return false;
    if (!verifyCrc(resp, 8)) return false;

    // Verify echo matches
    return (resp[2] == req[2] && resp[3] == req[3] &&
            resp[4] == req[4] && resp[5] == req[5]);
}

// Write a register only if it doesn't already hold the desired value.
// Returns true if the register now holds the desired value (whether we wrote or skipped).
static bool ensureRegister(uint16_t regAddr, uint16_t desired, const char* name) {
    uint16_t current;
    if (readRegister(regAddr, &current)) {
        if (current == desired) {
            Serial.printf("[MODBUS] %s (0x%04X) already %u - skipped\n", name, regAddr, desired);
            return true;
        }
        // Value differs, need to write
        bool ok = writeRegister(regAddr, desired);
        if (ok) {
            Serial.printf("[MODBUS] %s (0x%04X) %u -> %u\n", name, regAddr, current, desired);
        } else {
            Serial.printf("[MODBUS] FAILED to write %s (0x%04X) = %u\n", name, regAddr, desired);
        }
        return ok;
    }
    // Couldn't read - try writing blind
    bool ok = writeRegister(regAddr, desired);
    if (ok) {
        Serial.printf("[MODBUS] %s (0x%04X) = %u (read failed, wrote blind)\n", name, regAddr, desired);
    } else {
        Serial.printf("[MODBUS] FAILED to read or write %s (0x%04X)\n", name, regAddr);
    }
    return ok;
}

// ---- State ----

static volatile bool _connected = false;
static volatile bool _configured = false;  // true after initServo() succeeds
static ServoTelemetry _telemetry = {};

// Timing
static uint32_t _lastTelemetryMs = 0;

// Telemetry round-robin: cycle through different register reads each tick
// to spread the Modbus load across multiple loop() calls
static uint8_t _telemetryPhase = 0;

// ---- Public API implementation ----

namespace ModbusManager {

void setup() {
    modbusSerial.begin(MODBUS_BAUD, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    Serial.println("[MODBUS] UART2 initialized (57600 8E1 on GPIO16/17)");
    Serial.println("[MODBUS] Waiting for initServo() call after homing...");
}

void loop() {
    if (!_configured) return;

    uint32_t now = millis();

    // One Modbus read per loop iteration, round-robin across telemetry registers.
    // Each read takes ~5ms on success or ~200ms on timeout.
    // At 20ms intervals we get ~17Hz per register type (50Hz total across 3 phases).
    if (now - _lastTelemetryMs < 20) return;
    _lastTelemetryMs = now;

    switch (_telemetryPhase) {
        case 0: {
            // Read speed feedback + torque command in one batch (2 consecutive regs)
            uint16_t vals[2];
            if (readRegisters(REG_MON_SPEED_FB, 2, vals)) {
                _telemetry.speedFeedback = (int16_t)vals[0];
                _telemetry.torqueCommand = (int16_t)vals[1];
                _telemetry.valid = true;
                _connected = true;
            } else {
                _connected = false;
            }
            break;
        }
        case 1: {
            // Read torque feedback
            uint16_t val;
            if (readRegister(REG_MON_TORQUE_FB, &val)) {
                _telemetry.torqueFeedback = (int16_t)val;
                _connected = true;
            } else {
                _connected = false;
            }
            break;
        }
        case 2: {
            // Read speed command
            uint16_t val;
            if (readRegister(REG_MON_SPEED_CMD, &val)) {
                _telemetry.speedCommand = (int16_t)val;
                _connected = true;
            } else {
                _connected = false;
            }
            break;
        }
    }

    _telemetryPhase = (_telemetryPhase + 1) % 3;
}

bool initServo() {
    Serial.println("[MODBUS] Initializing servo compliance...");

    // First, verify we can talk to the servo by reading any register
    uint16_t testVal;
    bool canTalk = false;

    // Try reading speed feedback register (should work on any firmware version)
    for (int attempt = 0; attempt < 3; attempt++) {
        if (readRegister(REG_MON_SPEED_FB, &testVal)) {
            canTalk = true;
            break;
        }
        Serial.printf("[MODBUS] Connection attempt %d failed, retrying...\n", attempt + 1);
        delay(100);
    }

    if (!canTalk) {
        Serial.println("[MODBUS] Could not connect to servo. Compliance disabled.");
        Serial.println("[MODBUS] Check wiring: Board TX -> Servo RX, Board RX -> Servo TX, GND -> GND");
        _connected = false;
        _configured = false;
        return false;
    }

    _connected = true;
    Serial.printf("[MODBUS] Servo responding (speed_fb register returned %d)\n", (int16_t)testVal);

    // Configure gain switching for automatic compliance.
    // Each register is read first; if it already holds the desired value, the
    // write is skipped to avoid unnecessary EEPROM wear. On a fresh servo this
    // writes ~10 registers. On subsequent boots with persisted values, it writes zero.
    bool allOk = true;
    Serial.println("[MODBUS] Checking compliance config (read-before-write to avoid EEPROM wear)...");

    // Gain set 1 (firm) - don't overwrite if the servo already has reasonable values.
    // The user may have auto-tuned these via JMC software - we don't want to clobber that.
    // We only write our defaults if the current value is zero (unconfigured).
    uint16_t currentPosGain1 = 0, currentSpdGain1 = 0, currentSpdInt1 = 0;
    readRegister(REG_POS_GAIN_1, &currentPosGain1);
    readRegister(REG_SPD_GAIN_1, &currentSpdGain1);
    readRegister(REG_SPD_INTEGRAL_1, &currentSpdInt1);

    if (currentPosGain1 > 0) {
        Serial.printf("[MODBUS] Keeping existing firm pos gain: %.1f 1/S\n", currentPosGain1 / 10.0f);
    } else {
        allOk &= ensureRegister(REG_POS_GAIN_1, FIRM_POS_GAIN, "P02-00 pos_gain_1 (firm)");
    }
    if (currentSpdGain1 > 0) {
        Serial.printf("[MODBUS] Keeping existing firm spd gain: %.1f Hz\n", currentSpdGain1 / 10.0f);
    } else {
        allOk &= ensureRegister(REG_SPD_GAIN_1, FIRM_SPD_GAIN, "P02-10 spd_gain_1 (firm)");
    }
    if (currentSpdInt1 > 0) {
        Serial.printf("[MODBUS] Keeping existing firm spd integral: %.1f ms\n", currentSpdInt1 / 10.0f);
    } else {
        allOk &= ensureRegister(REG_SPD_INTEGRAL_1, FIRM_SPD_INT, "P02-11 spd_int_1 (firm)");
    }

    // Gain set 2 (soft) - always ensure these match our desired values
    allOk &= ensureRegister(REG_POS_GAIN_2, SOFT_POS_GAIN, "P02-01 pos_gain_2 (soft)");
    allOk &= ensureRegister(REG_SPD_GAIN_2, SOFT_SPD_GAIN, "P02-13 spd_gain_2 (soft)");
    allOk &= ensureRegister(REG_SPD_INTEGRAL_2, SOFT_SPD_INT, "P02-14 spd_int_2 (soft)");

    // Gain switching behavior
    allOk &= ensureRegister(REG_GAIN_SWITCH_MODE, COMPLIANCE_SWITCH_MODE, "P02-30 gain_switch_mode");
    allOk &= ensureRegister(REG_GAIN_SWITCH_LEVEL, COMPLIANCE_DEFAULT_LEVEL, "P02-31 gain_switch_level");
    allOk &= ensureRegister(REG_GAIN_SWITCH_HYST, COMPLIANCE_HYSTERESIS, "P02-32 gain_switch_hyst");
    allOk &= ensureRegister(REG_GAIN_SWITCH_DELAY, COMPLIANCE_SWITCH_DELAY, "P02-33 gain_switch_delay");

    _configured = allOk;

    if (allOk) {
        Serial.println("[MODBUS] Servo compliance ready");
        Serial.println("[MODBUS] Servo handles gain switching internally at 2kHz");
    } else {
        Serial.println("[MODBUS] WARNING: Some registers failed. Compliance may be partial.");
        Serial.println("[MODBUS] Check firmware version (V5 vs V6 have different register addresses).");
    }

    return allOk;
}

bool isConnected() {
    return _connected;
}

const ServoTelemetry& getTelemetry() {
    return _telemetry;
}

} // namespace ModbusManager
