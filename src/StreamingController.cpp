#include "StreamingController.h"

StreamingController::StreamingController()
    : _servo(nullptr)
    , _minStep(0)
    , _maxStep(0)
    , _maxStepPerSecond(1)
    , _maxStepAcceleration(1)
    , _strokeMin(0)
    , _strokeMax(0)
    , _commandQueue(nullptr)
    , _active(false)
    , _initialized(false)
    , _tickTask(nullptr)
    , _taskDoneSemaphore(nullptr)
{
}

void StreamingController::begin(
    FastAccelStepper* servo,
    int32_t minStep,
    int32_t maxStep,
    uint32_t maxStepPerSecond,
    uint32_t maxStepAcceleration
) {
    _servo = servo;
    _minStep = minStep;
    _maxStep = maxStep;
    _maxStepPerSecond = maxStepPerSecond;
    _maxStepAcceleration = maxStepAcceleration;

    // Default stroke range to full travel
    _strokeMin = minStep;
    _strokeMax = maxStep;

    // Create FreeRTOS queue for thread-safe command ingestion
    _commandQueue = xQueueCreate(STREAMING_QUEUE_SIZE, sizeof(StreamingCommand));

    // Semaphore for cooperative task shutdown
    _taskDoneSemaphore = xSemaphoreCreateBinary();

    _initialized = (_servo != nullptr && _commandQueue != nullptr && _taskDoneSemaphore != nullptr);
}

void StreamingController::updatePhysicalLimits(int32_t minStep, int32_t maxStep) {
    _minStep = minStep;
    _maxStep = maxStep;
    // Also update stroke limits to match physical limits by default
    _strokeMin = minStep;
    _strokeMax = maxStep;
}

void StreamingController::setStrokeLimits(int32_t minStep, int32_t maxStep) {
    // Constrain to physical limits
    _strokeMin = constrain(minStep, _minStep, _maxStep);
    _strokeMax = constrain(maxStep, _minStep, _maxStep);
    // Collapse to point if range is invalid
    if (_strokeMin > _strokeMax) {
        _strokeMax = _strokeMin;
    }
}

// =============================================================================
// SAFETY GUARDS
// These are the ONLY functions that directly command the servo.
// All trajectory math, all algorithms - their output goes through these guards.
// The guards don't trust ANY upstream code.
// =============================================================================

void StreamingController::safeMoveTo(int32_t pos) {
    if (!_servo) return;
    pos = constrain(pos, _minStep, _maxStep);
    _servo->moveTo(pos);
}

void StreamingController::safeSetSpeed(uint32_t speed) {
    if (!_servo) return;
    if (speed > _maxStepPerSecond) {
        speed = _maxStepPerSecond;
    }
    if (speed < 1) {
        speed = 1;
    }
    _servo->setSpeedInHz(speed);
}

void StreamingController::safeSetAcceleration(uint32_t accel) {
    if (!_servo) return;
    if (accel > _maxStepAcceleration) {
        accel = _maxStepAcceleration;
    }
    if (accel < 1) {
        accel = 1;
    }
    _servo->setAcceleration(accel);
}

// =============================================================================
// Public interface
// =============================================================================

void StreamingController::addTarget(uint8_t position_pct, uint32_t duration_ms, bool replace) {
    if (!_initialized) return;

    StreamingCommand cmd;
    cmd.position_pct = position_pct > 100 ? 100 : position_pct;
    cmd.duration_ms = duration_ms;
    cmd.replace = replace;

    // Non-blocking queue send - drop command if queue full rather than block
    xQueueSend(_commandQueue, &cmd, 0);
}

void StreamingController::start() {
    if (!_initialized) return;

    // Flush stale commands from before streaming started
    if (_commandQueue) {
        xQueueReset(_commandQueue);
    }

    _active = true;

    // Create dedicated processing task on Core 1 with priority 15
    if (_tickTask == nullptr) {
        xTaskCreatePinnedToCore(
            tickTaskWrapper,
            "StreamTick",
            4096,           // Stack size
            this,           // Parameter
            15,             // Priority
            &_tickTask,
            1               // Core 1
        );
    }
}

void StreamingController::stop() {
    if (!_initialized) return;

    // Signal task to exit
    _active = false;

    // Wait for task to finish (max 500ms - task checks _active every 10ms, so this is generous)
    if (_tickTask != nullptr) {
        if (xSemaphoreTake(_taskDoneSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Task exited cleanly and self-deleted
            _tickTask = nullptr;
        } else {
            // Task didn't respond in time. Force-delete is risky (could hold a spinlock
            // that deadlocks subsequent FastAccelStepper calls), but we can't leak the task.
            Serial.println("WARNING: StreamTick task did not exit cleanly, force deleting");
            vTaskDelete(_tickTask);
            _tickTask = nullptr;
            // Small delay to let any ISR referencing the deleted task settle
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Safe to cleanup -- task is guaranteed stopped
    if (_commandQueue) {
        xQueueReset(_commandQueue);
    }

    // Controlled deceleration, not violent forceStop()
    if (_servo) {
        _servo->stopMove();
    }
}

// =============================================================================
// Internal helpers
// =============================================================================

int32_t StreamingController::convertToSteps(uint8_t position_pct) {
    // Convert 0-100% to step position within stroke range
    int32_t range = _strokeMax - _strokeMin;
    int32_t pos = _strokeMin + (int32_t)(((int64_t)range * position_pct) / 100);

    // Final safety clamp
    return constrain(pos, _minStep, _maxStep);
}

// =============================================================================
// Execute-and-wait task (matches old StrokeEngine streaming model)
// =============================================================================

void StreamingController::tickTaskWrapper(void* params) {
    StreamingController* self = static_cast<StreamingController*>(params);
    self->_runTickTask();
}

void StreamingController::_runTickTask() {
    StreamingCommand cmd;
    int32_t lastPos = _servo ? _servo->getCurrentPosition() : 0;
    uint32_t cmdCount = 0;

    while (_active) {
        // Block waiting for next command (10ms timeout to check _active flag)
        if (xQueueReceive(_commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            cmdCount++;

            // Handle replace flag - stop current move and drain queue
            if (cmd.replace) {
                if (_servo) _servo->stopMove();
                StreamingCommand drain;
                while (xQueueReceive(_commandQueue, &drain, 0) == pdTRUE) {}
            }

            int32_t targetPos = convertToSteps(cmd.position_pct);
            int32_t distance = abs(targetPos - lastPos);

            if (distance > 0 && cmd.duration_ms > 0) {
                // Speed from StrokeEngine formula: peak speed for trapezoidal profile
                float time_s = constrain(cmd.duration_ms / 1000.0f, 0.02f, 120.0f);
                uint32_t speed = (uint32_t)(1.5f * distance / time_s);

                // Always use high acceleration. The per-move formula (3*speed/time)
                // assumes the motor starts from rest, but mid-streaming the motor is
                // almost never at rest. A short gentle move after a fast one would get
                // an accel too low to decelerate the motor's existing momentum, causing
                // step loss and physical drift past the keepout zone.
                uint32_t accel = _maxStepAcceleration / 2;
                if (accel < 1000) accel = 1000;

                safeSetSpeed(speed);
                safeSetAcceleration(accel);
                safeMoveTo(targetPos);

                // Log every 50th command for crash diagnostics
                // Log every command when heap is low
                uint32_t freeHeap = ESP.getFreeHeap();
                if (cmdCount % 50 == 0 || freeHeap < 20000) {
                    Serial.printf("[stream] #%u pos=%d->%d spd=%u acc=%u heap=%u q=%u\n",
                                  cmdCount, (int)lastPos, (int)targetPos,
                                  speed, accel, (unsigned)freeHeap,
                                  (unsigned)uxQueueMessagesWaiting(_commandQueue));
                }
            } else if (distance > 0) {
                // time=0 means move immediately at max speed
                safeSetSpeed(_maxStepPerSecond);
                safeSetAcceleration(_maxStepAcceleration);
                safeMoveTo(targetPos);
            }

            lastPos = targetPos;
        }
    }

    // Signal stop() that we're done before self-deleting
    xSemaphoreGive(_taskDoneSemaphore);
    vTaskDelete(nullptr);
}
