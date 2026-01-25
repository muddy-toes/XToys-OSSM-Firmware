#include "SerialManager.h"

namespace SerialManager {
    QueueHandle_t messageQueue = nullptr;
    void (*msgReceivedCallback)(String) = nullptr;
    unsigned long lastActivityTime = 0;

    static char lineBuffer[SERIAL_MAX_MESSAGE_LENGTH];
    static size_t lineIndex = 0;
    static bool skipUntilNewline = false;

    void setup(void (*callback)(String)) {
        msgReceivedCallback = callback;
        messageQueue = xQueueCreate(SERIAL_MESSAGE_QUEUE_SIZE, SERIAL_MAX_MESSAGE_LENGTH);
        lastActivityTime = 0;
        lineIndex = 0;
        skipUntilNewline = false;
        Serial.println("SerialManager initialized");
    }

    void loop() {
        while (Serial.available()) {
            char c = Serial.read();
            lastActivityTime = millis();

            if (c == '\n' || c == '\r') {
                if (!skipUntilNewline && lineIndex > 0) {
                    lineBuffer[lineIndex] = '\0';
                    if (messageQueue != nullptr) {
                        xQueueSend(messageQueue, lineBuffer, 0);
                    }
                }
                lineIndex = 0;
                skipUntilNewline = false;
            } else if (skipUntilNewline) {
                // Skip characters until we see newline
            } else if (lineIndex < SERIAL_MAX_MESSAGE_LENGTH - 1) {
                lineBuffer[lineIndex++] = c;
            } else {
                Serial.println("Serial buffer overflow - message dropped");
                skipUntilNewline = true;
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
