#pragma once

#include <Arduino.h>
#include <freertos/queue.h>
#include "config.h"

#define SERIAL_MESSAGE_QUEUE_SIZE 16
#define SERIAL_MAX_MESSAGE_LENGTH 512
#define SERIAL_ACTIVITY_TIMEOUT_MS 5000

namespace SerialManager {
    extern QueueHandle_t messageQueue;
    extern void (*msgReceivedCallback)(String);
    extern unsigned long lastActivityTime;

    void setup(void (*msgReceivedCallback)(String));
    void loop();
    void processQueue();
    bool isConnected();
};
