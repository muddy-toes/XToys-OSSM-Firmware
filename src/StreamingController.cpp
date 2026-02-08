#include "StreamingController.h"

StreamingController::StreamingController()
    : _servo(nullptr)
    , _minStep(0)
    , _maxStep(0)
    , _maxStepPerSecond(1)
    , _maxStepAcceleration(1)
    , _stepsPerMillimeter(1.0f)
    , _strokeMin(0)
    , _strokeMax(0)
    , _commandQueue(nullptr)
    , _lastDeadline(0)
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
    uint32_t maxStepAcceleration,
    float stepsPerMillimeter
) {
    _servo = servo;
    _minStep = minStep;
    _maxStep = maxStep;
    _maxStepPerSecond = maxStepPerSecond;
    _maxStepAcceleration = maxStepAcceleration;
    _stepsPerMillimeter = stepsPerMillimeter;

    // Default stroke range to full travel
    _strokeMin = minStep;
    _strokeMax = maxStep;

    // Create FreeRTOS queue for thread-safe command ingestion
    _commandQueue = xQueueCreate(STREAMING_QUEUE_SIZE, sizeof(StreamingCommand));

    // Semaphore for cooperative tick task shutdown
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
    if (!_servo) return;  // Null check
    // CRITICAL: Constrain position to physical limits
    // This line is the final defense - nothing bypasses it
    pos = constrain(pos, _minStep, _maxStep);
    _servo->moveTo(pos);
}

void StreamingController::safeSetSpeed(uint32_t speed) {
    if (!_servo) return;
    // CRITICAL: Clamp speed to maximum
    // Even if trajectory math overflows or bugs out, this catches it
    if (speed > _maxStepPerSecond) {
        speed = _maxStepPerSecond;
    }
    // Also ensure minimum speed of 1 to avoid division by zero elsewhere
    if (speed < 1) {
        speed = 1;
    }
    _servo->setSpeedInHz(speed);
}

void StreamingController::safeSetAcceleration(uint32_t accel) {
    if (!_servo) return;
    // CRITICAL: Clamp acceleration to maximum
    if (accel > _maxStepAcceleration) {
        accel = _maxStepAcceleration;
    }
    // Minimum acceleration of 1
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
    cmd.position_pct = position_pct > 100 ? 100 : position_pct;  // Clamp input
    cmd.duration_ms = duration_ms;
    cmd.replace = replace;

    // Non-blocking queue send - drop command if queue full rather than block
    // This is called from BLE/WebSocket callbacks which must not block
    xQueueSend(_commandQueue, &cmd, 0);
}

void StreamingController::start() {
    if (!_initialized) return;

    // Flush stale commands from before streaming started
    if (_commandQueue) {
        xQueueReset(_commandQueue);
    }
    _targets.clear();

    _active = true;
    _lastDeadline = 0;

    // Create dedicated tick task on Core 1 with priority 15
    // (below homing at 20, above normal tasks)
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

    // Wait for task to finish (max 200ms -- tick is 10ms, this is generous)
    if (_tickTask != nullptr) {
        if (xSemaphoreTake(_taskDoneSemaphore, pdMS_TO_TICKS(200)) != pdTRUE) {
            vTaskDelete(_tickTask);
        }
        _tickTask = nullptr;
    }

    // Safe to cleanup -- task is guaranteed stopped
    if (_commandQueue) {
        xQueueReset(_commandQueue);
    }
    _targets.clear();

    // Controlled deceleration, not violent forceStop()
    if (_servo) {
        _servo->stopMove();
    }

    _lastDeadline = 0;
}

void StreamingController::tick() {
    if (!_initialized || !_active) return;

    // 1. Drain incoming commands from FreeRTOS queue into internal buffer
    drainQueue();

    // 2. Remove expired targets
    pruneExpired();

    // 3. If no targets, nothing to do (motor will coast to stop on its own)
    if (_targets.isEmpty()) {
        return;
    }

    // 4. Get current state
    int32_t currentPos = _servo->getCurrentPosition();
    uint32_t now = millis();
    StreamingTarget current = _targets.first();

    // 5. Compute trajectory
    int32_t targetPos;
    uint32_t speed;
    uint32_t accel;

    if (current.deadline_ms > now) {
        // On schedule - compute profile to reach target on time
        computeOnTimeProfile(currentPos, current, now, &targetPos, &speed, &accel);
    } else {
        // Behind schedule - interpolate toward upcoming targets
        computeCatchupProfile(currentPos, now, &targetPos, &speed, &accel);
    }

    // 6. Issue commands through safety guards
    // These guards are the last line of defense - they don't trust any of the
    // trajectory math above
    // Note: FastAccelStepper handles direction reversal internally via RAMP_STATE_REVERSE
    safeSetSpeed(speed);
    safeSetAcceleration(accel);
    safeMoveTo(targetPos);
}

// =============================================================================
// Internal helpers
// =============================================================================

void StreamingController::drainQueue() {
    StreamingCommand cmd;

    // Drain all pending commands from the queue
    while (xQueueReceive(_commandQueue, &cmd, 0) == pdTRUE) {
        // Handle replace flag - clear existing targets
        if (cmd.replace) {
            _targets.clear();
            _lastDeadline = 0;
        }

        // Compute absolute deadline
        uint32_t now = millis();
        uint32_t deadline;
        if (_lastDeadline > now) {
            // Chain from last deadline
            deadline = _lastDeadline + cmd.duration_ms;
        } else {
            // Start fresh from now
            deadline = now + cmd.duration_ms;
        }
        // Skip this command if buffer is full (drop new, not old)
        // Don't update _lastDeadline so next command chains correctly
        if (_targets.isFull()) {
            continue;
        }
        _lastDeadline = deadline;

        // Convert position to steps
        int32_t pos_steps = convertToSteps(cmd.position_pct);

        // Add to internal buffer
        StreamingTarget target;
        target.position = pos_steps;
        target.deadline_ms = deadline;
        _targets.push(target);
    }
}

void StreamingController::pruneExpired() {
    uint32_t now = millis();

    // Remove targets whose deadlines have passed, but keep at least one
    // so we know where we're supposed to be heading
    while (_targets.size() > 1 && _targets.first().deadline_ms < now) {
        _targets.shift();
    }
}

int32_t StreamingController::convertToSteps(uint8_t position_pct) {
    // Convert 0-100% to step position within stroke range
    // Using 64-bit intermediate to avoid overflow
    int32_t range = _strokeMax - _strokeMin;
    int32_t pos = _strokeMin + (int32_t)(((int64_t)range * position_pct) / 100);

    // Final safety clamp (should be redundant but defense in depth)
    return constrain(pos, _minStep, _maxStep);
}

void StreamingController::computeOnTimeProfile(
    int32_t currentPos,
    const StreamingTarget& target,
    uint32_t now,
    int32_t* outTargetPos,
    uint32_t* outSpeed,
    uint32_t* outAccel
) {
    // Time remaining to deadline in milliseconds
    uint32_t timeRemaining_ms = target.deadline_ms - now;
    if (timeRemaining_ms < 1) timeRemaining_ms = 1;  // Avoid division by zero

    // Distance to target
    int32_t distance = target.position - currentPos;
    uint32_t absDistance = abs(distance);

    // Convert time to seconds for speed calculation
    float timeRemaining_s = timeRemaining_ms / 1000.0f;

    // For a trapezoidal profile, average speed is roughly distance/time
    // Peak speed is about 1.5x average for symmetric accel/decel
    // Using 64-bit to avoid overflow
    uint32_t avgSpeed = (uint32_t)(absDistance / timeRemaining_s);
    uint32_t peakSpeed = (avgSpeed * 3) / 2;  // 1.5x average

    // Acceleration for trapezoidal profile
    // With 1.5x peak speed and symmetric accel/decel, need 3v/t
    uint32_t accel = (uint32_t)((peakSpeed * 3) / timeRemaining_s);

    // Set outputs - safety guards will clamp these
    *outTargetPos = target.position;
    *outSpeed = peakSpeed;
    *outAccel = accel;
}

void StreamingController::computeCatchupProfile(
    int32_t currentPos,
    uint32_t now,
    int32_t* outTargetPos,
    uint32_t* outSpeed,
    uint32_t* outAccel
) {
    // We're behind schedule. The current target's deadline has passed.
    // Strategy: Look at where we should be heading and move there quickly.

    if (_targets.isEmpty()) {
        // No targets - hold position
        *outTargetPos = currentPos;
        *outSpeed = 1;
        *outAccel = 1;
        return;
    }

    StreamingTarget current = _targets.first();

    // If there's a next target, interpolate between current and next
    if (_targets.size() > 1) {
        StreamingTarget next = _targets[1];

        // How far past the current deadline are we?
        uint32_t lateness = now - current.deadline_ms;

        // Time between current and next deadlines
        uint32_t span = next.deadline_ms - current.deadline_ms;
        if (span < 1) span = 1;

        // Where "should" we be on the timeline between current and next?
        float progress = (float)lateness / (float)span;
        if (progress > 1.0f) progress = 1.0f;

        // Interpolate position
        int32_t interpPos = current.position +
            (int32_t)((next.position - current.position) * progress);

        // Aim a bit ahead of interpolated position to catch up (50ms ahead)
        float aheadProgress = progress + (50.0f / span);
        if (aheadProgress > 1.0f) aheadProgress = 1.0f;

        int32_t aheadPos = current.position +
            (int32_t)((next.position - current.position) * aheadProgress);

        *outTargetPos = aheadPos;

        // Move at high speed to catch up (80% of max)
        *outSpeed = (_maxStepPerSecond * 80) / 100;
        *outAccel = (_maxStepAcceleration * 80) / 100;
    } else {
        // Only one target (current) - just head to it at moderate speed
        *outTargetPos = current.position;
        *outSpeed = (_maxStepPerSecond * 60) / 100;
        *outAccel = (_maxStepAcceleration * 60) / 100;
    }
}

// =============================================================================
// Dedicated tick task
// =============================================================================

void StreamingController::tickTaskWrapper(void* params) {
    StreamingController* self = static_cast<StreamingController*>(params);
    self->_runTickTask();
}

void StreamingController::_runTickTask() {
    while (_active) {
        tick();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Signal stop() that we're done before self-deleting
    xSemaphoreGive(_taskDoneSemaphore);
    vTaskDelete(nullptr);
}
