#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Queue size for incoming commands from BLE/WebSocket/Serial
#define STREAMING_QUEUE_SIZE 16

// Incoming command structure (position as percentage, gets converted)
struct StreamingCommand {
    uint8_t position_pct;   // 0-100
    uint32_t duration_ms;   // duration for this move
    bool replace;           // clear buffer first?
};

class StreamingController {
public:
    StreamingController();

    // Setup - must be called before use
    // Requires servo pointer and all safety limits
    void begin(
        FastAccelStepper* servo,
        int32_t minStep,
        int32_t maxStep,
        uint32_t maxStepPerSecond,
        uint32_t maxStepAcceleration
    );

    // Update physical limits (call after homing when actual travel is known)
    void updatePhysicalLimits(int32_t minStep, int32_t maxStep);

    // Update stroke limits (called when stroke/depth changes)
    void setStrokeLimits(int32_t minStep, int32_t maxStep);

    // Add a target - thread-safe, can be called from any core
    // position_pct: 0-100
    // duration_ms: time to reach this position
    // replace: if true, clear existing buffer first
    void addTarget(uint8_t position_pct, uint32_t duration_ms, bool replace = false);

    // Start streaming mode
    void start();

    // Stop immediately - clears queue and halts motor
    void stop();

    // Query state
    bool isActive() const { return _active; }

private:
    // =========================================================
    // SAFETY GUARDS - ALL motor commands go through these
    // These are the ONLY functions that touch the servo directly
    // =========================================================

    void safeMoveTo(int32_t pos);
    void safeSetSpeed(uint32_t speed);
    void safeSetAcceleration(uint32_t accel);

    // =========================================================
    // Internal helpers
    // =========================================================

    // Convert position percentage to steps using current limits
    int32_t convertToSteps(uint8_t position_pct);

    // =========================================================
    // Member variables
    // =========================================================

    FastAccelStepper* _servo;

    // Safety limits - these are the absolute constraints
    // volatile: written from main loop, read from task
    volatile int32_t _minStep;
    volatile int32_t _maxStep;
    uint32_t _maxStepPerSecond;
    uint32_t _maxStepAcceleration;

    // Current stroke range for position conversion
    // volatile: written from main loop, read from task
    volatile int32_t _strokeMin;
    volatile int32_t _strokeMax;

    // FreeRTOS queue for thread-safe command ingestion
    QueueHandle_t _commandQueue;

    // State
    volatile bool _active;
    bool _initialized;

    // Dedicated processing task
    TaskHandle_t _tickTask;
    SemaphoreHandle_t _taskDoneSemaphore;
    static void tickTaskWrapper(void* params);
    void _runTickTask();
};
