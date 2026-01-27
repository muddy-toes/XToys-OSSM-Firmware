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
    , _currentDirection(0)
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

    _initialized = (_servo != nullptr && _commandQueue != nullptr);

    if (!_initialized) {
        Serial.println("StreamingController: Failed to initialize!");
    }
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
    // CRITICAL: Constrain position to physical limits
    // This line is the final defense - nothing bypasses it
    pos = constrain(pos, _minStep, _maxStep);
    _servo->moveTo(pos);
}

void StreamingController::safeSetSpeed(uint32_t speed) {
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

    _active = true;
    _lastDeadline = 0;
    _currentDirection = 0;
}

void StreamingController::stop() {
    if (!_initialized) return;

    _active = false;

    // Clear the command queue
    xQueueReset(_commandQueue);

    // Clear internal buffer
    _targets.clear();

    // IMMEDIATE stop - not controlled deceleration
    _servo->forceStop();

    _lastDeadline = 0;
    _currentDirection = 0;
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

    // 6. Handle direction reversal
    // FastAccelStepper silently ignores moveTo commands that would reverse direction
    // We need to issue stopMove() first and wait for it to slow down
    if (needsReversal(currentPos, targetPos)) {
        _servo->stopMove();  // Controlled deceleration
        return;  // Will issue moveTo on next tick when speed is low enough
    }

    // 7. Issue commands through safety guards
    // These guards are the last line of defense - they don't trust any of the
    // trajectory math above
    safeSetSpeed(speed);
    safeSetAcceleration(accel);
    safeMoveTo(targetPos);

    // Update direction tracking
    if (targetPos > currentPos) {
        _currentDirection = 1;
    } else if (targetPos < currentPos) {
        _currentDirection = -1;
    }
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
        _lastDeadline = deadline;

        // Convert position to steps
        int32_t pos_steps = convertToSteps(cmd.position_pct);

        // Add to internal buffer (drops oldest if full)
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

bool StreamingController::needsReversal(int32_t currentPos, int32_t targetPos) {
    // Check if motor is currently moving
    if (!_servo->isRunning()) {
        return false;  // Not moving, no reversal issue
    }

    // Determine target direction
    int8_t targetDirection = 0;
    if (targetPos > currentPos) {
        targetDirection = 1;
    } else if (targetPos < currentPos) {
        targetDirection = -1;
    }

    // If target is in opposite direction from current movement, we need to reverse
    // But only if we're actually moving significantly
    if (_currentDirection != 0 && targetDirection != 0 && _currentDirection != targetDirection) {
        // Check current speed - if very slow, we can just change direction
        int32_t currentSpeed = _servo->getCurrentSpeedInMilliHz(true);
        if (abs(currentSpeed) < 1000) {  // Less than 1 step/sec
            return false;  // Slow enough to just change direction
        }
        return true;  // Need to decelerate first
    }

    return false;
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

    // Acceleration to reach peak speed in half the time
    // a = v / (t/2) = 2v/t
    uint32_t accel = (uint32_t)((peakSpeed * 2) / timeRemaining_s);

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
