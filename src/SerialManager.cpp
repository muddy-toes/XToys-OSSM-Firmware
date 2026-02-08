#include "SerialManager.h"

namespace SerialManager {
    QueueHandle_t messageQueue = nullptr;
    void (*msgReceivedCallback)(String) = nullptr;
    unsigned long lastActivityTime = 0;

    static char msgBuffer[SERIAL_MAX_MESSAGE_LENGTH];
    static size_t bufIndex = 0;
    static int bracketDepth = 0;
    static bool overflow = false;

    void setup(void (*callback)(String)) {
        msgReceivedCallback = callback;
        messageQueue = xQueueCreate(SERIAL_MESSAGE_QUEUE_SIZE, SERIAL_MAX_MESSAGE_LENGTH);
        lastActivityTime = 0;
        bufIndex = 0;
        bracketDepth = 0;
        overflow = false;
    }

    void loop() {
        while (Serial.available()) {
            char c = Serial.read();
            lastActivityTime = millis();

            // Ignore whitespace outside of a message
            if (bracketDepth == 0 && (c == '\n' || c == '\r' || c == ' ')) {
                continue;
            }

            // Track JSON array bracket depth
            if (c == '[') {
                if (bracketDepth == 0) {
                    // Start of new message
                    bufIndex = 0;
                    overflow = false;
                }
                bracketDepth++;
            }

            // Only buffer while inside brackets
            if (bracketDepth > 0) {
                if (!overflow && bufIndex < SERIAL_MAX_MESSAGE_LENGTH - 1) {
                    msgBuffer[bufIndex++] = c;
                } else {
                    overflow = true;
                }

                if (c == ']') {
                    bracketDepth--;
                    if (bracketDepth == 0) {
                        // Complete message
                        if (!overflow) {
                            msgBuffer[bufIndex] = '\0';
                            if (messageQueue != nullptr) {
                                xQueueSend(messageQueue, msgBuffer, 0);
                            }
                        }
                        bufIndex = 0;
                        overflow = false;
                    }
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
};
