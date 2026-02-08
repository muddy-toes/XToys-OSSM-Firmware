#include "MotorController.h"
#include "config.h"
#include <Arduino.h>

// Static member definition - single engine instance shared across all controllers
FastAccelStepperEngine MotorController::_engine = FastAccelStepperEngine();

void MotorController::begin(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                            float stepsPerMm, bool invertDir, bool enableActiveLow) {
    // Store steps per mm for conversions
    _stepsPerMm = stepsPerMm;

    // Initialize the FastAccelStepper engine (idempotent - safe to call multiple times)
    _engine.init();

    // Connect stepper to the step pin
    _servo = _engine.stepperConnectToPin(stepPin);

    if (_servo) {
        // Configure direction pin with optional inversion
        _servo->setDirectionPin(dirPin, invertDir);

        // Configure enable pin with active-low option
        _servo->setEnablePin(enablePin, enableActiveLow);

        // Disable auto-enable - we control enable manually for safety
        _servo->setAutoEnable(false);

        // Start with outputs disabled
        _servo->disableOutputs();

        // Set default speed and acceleration limits
        // These use the MAX_SPEED from config.h (2000 mm/s) and a reasonable default accel
        _maxStepPerSec = static_cast<uint32_t>(MAX_SPEED * _stepsPerMm);
        _maxStepAccel = static_cast<uint32_t>(100000.0f * _stepsPerMm);  // 100000 mm/s^2 default

        // Apply limits to the stepper
        _servo->setSpeedInHz(_maxStepPerSec);
        _servo->setAcceleration(_maxStepAccel);

    }

    // Start in disabled state - homing must be performed before motion
    _state = MOTOR_DISABLED;
}

void MotorController::disable() {
    // Set abort flag to stop any running homing tasks
    _abortHoming = true;
    stopPattern();

    if (_servo) {
        _servo->forceStop();
        _servo->disableOutputs();
    }
    _state = MOTOR_DISABLED;
}

void MotorController::moveToMax(float speed, bool blocking) {
    if (!_servo || _state != MOTOR_READY) return;

    uint32_t speedHz = static_cast<uint32_t>(speed * _stepsPerMm);
    _servo->setSpeedInHz(speedHz);
    _servo->moveTo(_maxStep);

    if (blocking) {
        while (_servo->isRunning()) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }

    // Restore normal speed
    _servo->setSpeedInHz(_maxStepPerSec);
}

MotorState MotorController::getState() {
    return _state;
}

FastAccelStepper* MotorController::getServo() {
    return _servo;
}

int32_t MotorController::getMinStep() {
    return _minStep;
}

int32_t MotorController::getMaxStep() {
    return _maxStep;
}

uint32_t MotorController::getMaxStepPerSecond() {
    return _maxStepPerSec;
}

uint32_t MotorController::getMaxStepAcceleration() {
    return _maxStepAccel;
}

float MotorController::getStepsPerMillimeter() {
    return _stepsPerMm;
}

int MotorController::getDepthPercent() {
    if (!_servo || _maxStep <= _minStep) {
        return 0;
    }

    int32_t currentPos = _servo->getCurrentPosition();

    // Clamp to valid range
    if (currentPos <= _minStep) {
        return 0;
    }
    if (currentPos >= _maxStep) {
        return 100;
    }

    // Calculate percentage: (currentPos - minStep) * 100 / (maxStep - minStep)
    return static_cast<int>((currentPos - _minStep) * 100 / (_maxStep - _minStep));
}

void MotorController::setPhysicalTravel(float mm) {
    _physicalTravel = mm;

    // Recalculate step limits: keepout at each end
    int32_t keepoutSteps = static_cast<int32_t>(_keepout * _stepsPerMm);
    int32_t totalSteps = static_cast<int32_t>(_physicalTravel * _stepsPerMm);
    _minStep = keepoutSteps;
    _maxStep = totalSteps - keepoutSteps;
    if (_maxStep < _minStep) {
        _maxStep = _minStep;
    }
}

void MotorController::setMaxSpeed(float mmPerSec) {
    // Convert mm/s to steps/s
    _maxStepPerSec = static_cast<uint32_t>(mmPerSec * _stepsPerMm + 0.5f);

    // Apply to stepper if initialized
    if (_servo) {
        _servo->setSpeedInHz(_maxStepPerSec);
    }
}

void MotorController::setMaxAcceleration(float mmPerSecSq) {
    // Convert mm/s^2 to steps/s^2
    _maxStepAccel = static_cast<uint32_t>(mmPerSecSq * _stepsPerMm + 0.5f);

    // Apply to stepper if initialized
    if (_servo) {
        _servo->setAcceleration(_maxStepAccel);
    }
}

// ============================================================================
// Helper methods
// ============================================================================

float MotorController::_getAnalogAveragePercent(int pin, int samples) {
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
    }
    float average = sum / samples;
    // ESP32 ADC is 12-bit (0-4095)
    return 100.0f * average / 4096.0f;
}

// ============================================================================
// Homing methods
// ============================================================================

void MotorController::homeEndstop(uint8_t endstopPin, bool activeLow, bool homeToBack,
                                  float speed, void (*callback)(bool)) {
    if (!_servo) {
        if (callback) callback(false);
        return;
    }

    if (_state == MOTOR_HOMING) {
        if (callback) callback(false);
        return;
    }

    // Reset abort flag before starting
    _abortHoming = false;

    // Store parameters for task access
    _homingCallback = callback;
    _homingEndstopPin = endstopPin;
    _homingActiveLow = activeLow;
    _homingToBack = homeToBack;
    _homingSpeed = speed;
    _state = MOTOR_HOMING;

    // Create homing task on core 1, priority 20
    xTaskCreatePinnedToCore(
        endstopHomingTaskWrapper,
        "endstopHoming",
        4096,           // Stack size
        this,           // Pass this pointer
        20,             // Priority
        &_homingTask,
        1               // Core 1
    );
}

void MotorController::homeManual(float rodLength) {
    if (!_servo) return;
    if (_state == MOTOR_HOMING) return;
    if (rodLength < 10.0f || rodLength > 500.0f) return;

    // Enable motor
    _servo->enableOutputs();

    // Position 0 = fully retracted (back hard stop)
    _servo->forceStopAndNewPosition(0);

    // rodLength is the total physical travel between hard stops (same meaning as sensorless)
    // Keepout is enforced: 5mm buffer at each end
    _physicalTravel = rodLength;
    int32_t keepoutSteps = static_cast<int32_t>(_keepout * _stepsPerMm);
    int32_t totalSteps = static_cast<int32_t>(_physicalTravel * _stepsPerMm);
    _minStep = keepoutSteps;               // 5mm from back hard stop
    _maxStep = totalSteps - keepoutSteps;  // 5mm from front hard stop

    _state = MOTOR_READY;
}

void MotorController::homeSensorless(uint8_t currentPin, float threshold, float speed,
                                     float maxTravel, void (*callback)(bool)) {
    if (!_servo) {
        if (callback) callback(false);
        return;
    }

    if (_state == MOTOR_HOMING) {
        if (callback) callback(false);
        return;
    }

    // Reset abort flag before starting
    _abortHoming = false;

    // Store parameters for task access
    _homingCallback = callback;
    _homingCurrentPin = currentPin;
    _homingThreshold = threshold;
    _homingSpeed = speed;
    _homingMaxTravel = maxTravel;
    _state = MOTOR_HOMING;

    // Create homing task on core 1, priority 20
    xTaskCreatePinnedToCore(
        sensorlessHomingTaskWrapper,
        "sensorlessHoming",
        4096,           // Stack size
        this,           // Pass this pointer
        20,             // Priority
        &_homingTask,
        1               // Core 1
    );
}

// ============================================================================
// Static wrapper functions for FreeRTOS tasks
// ============================================================================

void MotorController::endstopHomingTaskWrapper(void* params) {
    MotorController* self = static_cast<MotorController*>(params);
    self->_runEndstopHomingTask();
    self->_homingTask = nullptr;
    vTaskDelete(nullptr);
}

void MotorController::sensorlessHomingTaskWrapper(void* params) {
    MotorController* self = static_cast<MotorController*>(params);
    self->_runSensorlessHomingTask();
    self->_homingTask = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// Internal homing task implementations
// ============================================================================

void MotorController::_runEndstopHomingTask() {
    // Enable motor
    _servo->enableOutputs();

    // Set homing speed (convert mm/s to steps/s)
    uint32_t homingSpeedHz = static_cast<uint32_t>(_homingSpeed * _stepsPerMm);
    _servo->setSpeedInHz(homingSpeedHz);
    _servo->setAcceleration(_maxStepAccel / 100);  // Gentle acceleration for homing (1000 mm/s²)

    // Configure endstop pin
    pinMode(_homingEndstopPin, INPUT_PULLUP);

    // Check if already at endstop - if so, back off first
    bool endstopActive = (digitalRead(_homingEndstopPin) == (_homingActiveLow ? LOW : HIGH));
    if (endstopActive) {
        // Move away from endstop by 2x keepout
        int32_t backoffSteps = static_cast<int32_t>(2.0f * _keepout * _stepsPerMm);
        _servo->move(_homingToBack ? backoffSteps : -backoffSteps);
        while (_servo->isRunning() && !_abortHoming) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        // Verify we're clear of endstop
        endstopActive = (digitalRead(_homingEndstopPin) == (_homingActiveLow ? LOW : HIGH));
        if (endstopActive && !_abortHoming) {
            _servo->forceStop();
            _servo->disableOutputs();
            _state = MOTOR_DISABLED;
            if (_homingCallback) _homingCallback(false);
            return;
        }
    }

    // Check for abort before continuing
    if (_abortHoming) {
        _servo->forceStop();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // Run toward endstop (direction depends on homeToBack)
    if (_homingToBack) {
        _servo->runBackward();
    } else {
        _servo->runForward();
    }

    // Poll endstop until triggered, with abort check
    bool found = false;
    unsigned long startTime = millis();
    const unsigned long timeout = 30000;  // 30 second timeout

    while (!found && !_abortHoming && (millis() - startTime < timeout)) {
        endstopActive = (digitalRead(_homingEndstopPin) == (_homingActiveLow ? LOW : HIGH));
        if (endstopActive) {
            found = true;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (found && !_abortHoming) {
        // Position 0 = fully retracted (back hard stop, where endstop triggered)
        _servo->forceStopAndNewPosition(0);

        // Same coordinate system as manual/sensorless: keepout at each end
        int32_t keepoutSteps = static_cast<int32_t>(_keepout * _stepsPerMm);
        int32_t totalSteps = static_cast<int32_t>(_physicalTravel * _stepsPerMm);
        _minStep = keepoutSteps;               // 5mm from back hard stop
        _maxStep = totalSteps - keepoutSteps;  // 5mm from front hard stop

        // Validate step limits
        if (_minStep >= _maxStep) {
            _servo->disableOutputs();
            _state = MOTOR_DISABLED;
            if (_homingCallback) _homingCallback(false);
            return;
        }

        // Move to minStep (safe boundary, away from endstop)
        _servo->moveTo(_minStep);
        while (_servo->isRunning() && !_abortHoming) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        if (_abortHoming) {
            _servo->forceStop();
            _state = MOTOR_DISABLED;
            if (_homingCallback) _homingCallback(false);
            return;
        }

        _state = MOTOR_READY;

        if (_homingCallback) {
            _homingCallback(true);
        }
    } else {
        // Homing failed - disable motor
        _servo->forceStop();
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;

        if (_homingCallback) {
            _homingCallback(false);
        }
    }

    // Restore normal speed settings
    _servo->setSpeedInHz(_maxStepPerSec);
    _servo->setAcceleration(_maxStepAccel);
}

void MotorController::_runSensorlessHomingTask() {
    const unsigned long timeout = 30000;  // 30 second timeout per phase

    // Disable motor briefly in case we're against a hard stop
    _servo->disableOutputs();
    vTaskDelay(600 / portTICK_PERIOD_MS);

    if (_abortHoming) {
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // Re-enable and wait for servo to stabilize
    _servo->enableOutputs();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Read baseline current AFTER motor is enabled (more accurate)
    float baselineCurrent = _getAnalogAveragePercent(_homingCurrentPin, 1000);

    // Set homing speed (convert mm/s to steps/s) - use slow speed for reliable detection
    uint32_t homingSpeedHz = static_cast<uint32_t>(_homingSpeed * _stepsPerMm);
    _servo->setSpeedInHz(homingSpeedHz);
    _servo->setAcceleration(_maxStepAccel / 100);  // Gentle acceleration (1000 mm/s²)

    // === Phase 1: Run forward to find max position ===
    _servo->runForward();

    unsigned long startTime = millis();
    float currentReading = _getAnalogAveragePercent(_homingCurrentPin, 200) - baselineCurrent;
    while (currentReading < _homingThreshold && !_abortHoming && (millis() - startTime < timeout)) {
        if (_homingMaxTravel > 0) {
            float distanceTraveled = abs(_servo->getCurrentPosition()) / _stepsPerMm;
            if (distanceTraveled > _homingMaxTravel) {
                _abortHoming = true;
                break;
            }
        }
        currentReading = _getAnalogAveragePercent(_homingCurrentPin, 200) - baselineCurrent;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (_abortHoming || (millis() - startTime >= timeout)) {
        _servo->forceStop();
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // Found forward limit - this becomes position 0 (max)
    _servo->forceStopAndNewPosition(0);

    // Wait for current to settle
    vTaskDelay(300 / portTICK_PERIOD_MS);

    if (_abortHoming) {
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // === Phase 2: Run backward to find min position ===
    _servo->runBackward();

    startTime = millis();
    currentReading = _getAnalogAveragePercent(_homingCurrentPin, 200) - baselineCurrent;
    while (currentReading < _homingThreshold && !_abortHoming && (millis() - startTime < timeout)) {
        if (_homingMaxTravel > 0) {
            float distanceTraveled = abs(_servo->getCurrentPosition()) / _stepsPerMm;
            if (distanceTraveled > _homingMaxTravel) {
                _abortHoming = true;
                break;
            }
        }
        currentReading = _getAnalogAveragePercent(_homingCurrentPin, 200) - baselineCurrent;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (_abortHoming || (millis() - startTime >= timeout)) {
        _servo->forceStop();
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // Calculate measured travel from position (negative value = distance traveled backward)
    float measuredTravel = abs(_servo->getCurrentPosition()) / _stepsPerMm;

    // Position 0 = fully retracted (back hard stop, where we are now)
    _servo->forceStopAndNewPosition(0);

    // Same coordinate system as manual/endstop: keepout at each end
    _physicalTravel = measuredTravel;
    int32_t keepoutSteps = static_cast<int32_t>(_keepout * _stepsPerMm);
    int32_t totalSteps = static_cast<int32_t>(_physicalTravel * _stepsPerMm);
    _minStep = keepoutSteps;               // 5mm from back hard stop
    _maxStep = totalSteps - keepoutSteps;  // 5mm from front hard stop

    // Validate step limits
    if (_minStep >= _maxStep) {
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    // Move to minStep (safe boundary, away from back hard stop)
    _servo->moveTo(_minStep);
    while (_servo->isRunning() && !_abortHoming) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    if (_abortHoming) {
        _servo->forceStop();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }

    _state = MOTOR_READY;

    // Restore normal speed settings
    _servo->setSpeedInHz(_maxStepPerSec);
    _servo->setAcceleration(_maxStepAccel);

    if (_homingCallback) {
        _homingCallback(true);
    }
}

// ============================================================================
// Pattern (speed mode) - simple oscillation between two positions
// ============================================================================

void MotorController::startPattern(int32_t strokeMin, int32_t strokeMax, float speedStrokesPerMin) {
    if (!_servo || _state != MOTOR_READY) return;

    stopPattern();

    _patternMin = constrain(strokeMin, _minStep, _maxStep);
    _patternMax = constrain(strokeMax, _minStep, _maxStep);
    _patternSpeed = speedStrokesPerMin;
    _patternActive = true;

    xTaskCreatePinnedToCore(
        patternTaskWrapper,
        "Pattern",
        4096,
        this,
        15,             // Same priority as streaming tick task
        &_patternTask,
        1               // Core 1
    );
}

void MotorController::setPatternSpeed(float speedStrokesPerMin) {
    _patternSpeed = speedStrokesPerMin;
    _patternGen++;
}

void MotorController::setPatternLimits(int32_t strokeMin, int32_t strokeMax) {
    _patternMin = constrain(strokeMin, _minStep, _maxStep);
    _patternMax = constrain(strokeMax, _minStep, _maxStep);
    _patternGen++;
}

void MotorController::stopPattern() {
    _patternActive = false;
    if (_patternTask != nullptr) {
        // Give the task 200ms to exit cleanly
        for (int i = 0; i < 20 && _patternTask != nullptr; i++) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        if (_patternTask != nullptr) {
            vTaskDelete(_patternTask);
            _patternTask = nullptr;
        }
    }
}

void MotorController::patternTaskWrapper(void* params) {
    MotorController* self = static_cast<MotorController*>(params);
    self->_runPatternTask();
    self->_patternTask = nullptr;
    vTaskDelete(nullptr);
}

void MotorController::_runPatternTask() {
    bool movingToMax = true;
    uint32_t lastGen = _patternGen;

    while (_patternActive) {
        float speed = _patternSpeed;
        int32_t strokeRange = _patternMax - _patternMin;

        if (strokeRange <= 0 || speed <= 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            lastGen = _patternGen;
            continue;
        }

        // Convert strokes-per-minute to steps-per-second
        // One stroke = one full round trip (min->max->min)
        // Half-stroke (min->max or max->min) takes 60/(2*spm) seconds
        float halfStrokeTimeSec = 60.0f / (2.0f * speed);
        uint32_t stepsPerSec = static_cast<uint32_t>(strokeRange / halfStrokeTimeSec);
        if (stepsPerSec < 1) stepsPerSec = 1;
        if (stepsPerSec > _maxStepPerSec) stepsPerSec = _maxStepPerSec;

        // Acceleration: reach full speed within ~25% of the stroke
        uint32_t accel = stepsPerSec * 4;
        if (accel > _maxStepAccel) accel = _maxStepAccel;
        if (accel < 1000) accel = 1000;

        _servo->setSpeedInHz(stepsPerSec);
        _servo->setAcceleration(accel);

        int32_t target = movingToMax ? _patternMax : _patternMin;
        _servo->moveTo(target);
        lastGen = _patternGen;

        // Wait for move to complete, speed change, or stopped
        while (_servo->isRunning() && _patternActive && lastGen == _patternGen) {
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        // Speed changed mid-stroke - recompute without flipping direction
        if (lastGen != _patternGen) {
            continue;
        }

        movingToMax = !movingToMax;
    }
}
