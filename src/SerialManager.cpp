#include "SerialManager.h"

namespace SerialManager {
    QueueHandle_t messageQueue = nullptr;
    void (*msgReceivedCallback)(String) = nullptr;
    unsigned long lastActivityTime = 0;

    static char msgBuffer[SERIAL_MAX_MESSAGE_LENGTH];
    static size_t bufIndex = 0;
    static int bracketDepth = 0;
    static bool overflow = false;
    static bool inString = false;
    static bool escaped = false;

    static void resetFraming() {
        bufIndex = 0;
        bracketDepth = 0;
        overflow = false;
        inString = false;
        escaped = false;
    }

    void setup(void (*callback)(String)) {
        msgReceivedCallback = callback;
        messageQueue = xQueueCreate(SERIAL_MESSAGE_QUEUE_SIZE, SERIAL_MAX_MESSAGE_LENGTH);
        lastActivityTime = 0;
        resetFraming();
    }

    void loop() {
        // Drop a stalled partial message (interrupted transmission, boot
        // garbage containing a stray '[') rather than wedging the parser
        if (bracketDepth > 0 && lastActivityTime != 0 &&
            (millis() - lastActivityTime) > SERIAL_FRAME_TIMEOUT_MS) {
            resetFraming();
        }

        while (Serial.available()) {
            char c = Serial.read();
            lastActivityTime = millis();

            // Ignore everything between messages until an array starts
            if (bracketDepth == 0) {
                if (c != '[') continue;
                resetFraming();
                bracketDepth = 1;
                msgBuffer[bufIndex++] = c;
                continue;
            }

            // Inside a message - buffer the character
            if (!overflow && bufIndex < SERIAL_MAX_MESSAGE_LENGTH - 1) {
                msgBuffer[bufIndex++] = c;
            } else {
                overflow = true;
            }

            // Track JSON string state so brackets inside string values
            // (e.g. WiFi passwords) don't corrupt the framing
            if (inString) {
                if (escaped) {
                    escaped = false;
                } else if (c == '\\') {
                    escaped = true;
                } else if (c == '"') {
                    inString = false;
                }
            } else if (c == '"') {
                inString = true;
            } else if (c == '[') {
                bracketDepth++;
            } else if (c == ']') {
                bracketDepth--;
                if (bracketDepth == 0) {
                    // Complete message
                    if (!overflow) {
                        msgBuffer[bufIndex] = '\0';
                        if (messageQueue != nullptr) {
                            xQueueSend(messageQueue, msgBuffer, 0);
                        }
                    }
                    resetFraming();
                }
            }
        }
    }

    void processQueue() {
        if (messageQueue == nullptr || msgReceivedCallback == nullptr) return;

        char buffer[SERIAL_MAX_MESSAGE_LENGTH];
        while (xQueueReceive(messageQueue, buffer, 0) == pdTRUE) {
            msgReceivedCallback(String(buffer));
        }
    }

    bool isConnected() {
        if (lastActivityTime == 0) return false;
        return (millis() - lastActivityTime) < SERIAL_ACTIVITY_TIMEOUT_MS;
    }
}
