#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <CircularBuffer.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Buffer sizes
#define STREAMING_QUEUE_SIZE 16      // FreeRTOS queue from other cores
#define STREAMING_BUFFER_SIZE 16     // Internal circular buffer (~500ms at 30Hz)

// Target structure for internal buffer (position already converted to steps)
struct StreamingTarget {
    int32_t position;       // position in steps
    uint32_t deadline_ms;   // absolute millis() deadline
};

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
        uint32_t maxStepAcceleration,
        float stepsPerMillimeter
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

    // Main tick function - call from main loop every ~10ms
    void tick();

    // Query state
    bool isActive() const { return _active; }
    size_t queueDepth() const { return _targets.size(); }

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

    // Drain FreeRTOS queue into internal buffer
    void drainQueue();

    // Remove targets whose deadlines have passed
    void pruneExpired();

    // Convert position percentage to steps using current limits
    int32_t convertToSteps(uint8_t position_pct);

    // Compute motion profile when on schedule
    void computeOnTimeProfile(
        int32_t currentPos,
        const StreamingTarget& target,
        uint32_t now,
        int32_t* outTargetPos,
        uint32_t* outSpeed,
        uint32_t* outAccel
    );

    // Compute motion profile when behind schedule
    void computeCatchupProfile(
        int32_t currentPos,
        uint32_t now,
        int32_t* outTargetPos,
        uint32_t* outSpeed,
        uint32_t* outAccel
    );

    // =========================================================
    // Member variables
    // =========================================================

    FastAccelStepper* _servo;

    // Safety limits - these are the absolute constraints
    int32_t _minStep;
    int32_t _maxStep;
    uint32_t _maxStepPerSecond;
    uint32_t _maxStepAcceleration;
    float _stepsPerMillimeter;

    // Current stroke range for position conversion
    int32_t _strokeMin;  // min position in steps (depth - stroke)
    int32_t _strokeMax;  // max position in steps (depth)

    // FreeRTOS queue for thread-safe command ingestion
    QueueHandle_t _commandQueue;

    // Internal target buffer
    CircularBuffer<StreamingTarget, STREAMING_BUFFER_SIZE> _targets;

    // Deadline tracking for cumulative calculation
    uint32_t _lastDeadline;

    // State
    volatile bool _active;
    bool _initialized;

    // Dedicated tick task
    TaskHandle_t _tickTask;
    SemaphoreHandle_t _taskDoneSemaphore;
    static void tickTaskWrapper(void* params);
    void _runTickTask();
};
