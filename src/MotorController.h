#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum MotorState {
    MOTOR_DISABLED,
    MOTOR_HOMING,
    MOTOR_READY
};

class MotorController {
public:
    // Initialization
    void begin(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
               float stepsPerMm, bool invertDir, bool enableActiveLow);

    // Homing methods
    void homeEndstop(uint8_t endstopPin, bool activeLow, bool homeToBack, float speed,
                     void (*callback)(bool));
    void homeManual(float rodLength);
    void homeSensorless(uint8_t currentPin, float threshold, float speed,
                        float maxTravel, void (*callback)(bool));

    // State management
    MotorState getState();
    void disable();

    // Motion commands
    void moveToMax(float speed, bool blocking = false);

    // Speed mode (oscillation pattern)
    void startPattern(int32_t strokeMin, int32_t strokeMax, float speedStrokesPerMin);
    void setPatternSpeed(float speedStrokesPerMin);
    void setPatternLimits(int32_t strokeMin, int32_t strokeMax);
    void stopPattern();

    // Position info (needed by StreamingController)
    FastAccelStepper* getServo();
    int32_t getMinStep();
    int32_t getMaxStep();
    uint32_t getMaxStepPerSecond();
    uint32_t getMaxStepAcceleration();
    float getStepsPerMillimeter();
    int getDepthPercent();

    // Configuration
    void setPhysicalTravel(float mm);
    void setMaxSpeed(float mmPerSec);
    void setMaxAcceleration(float mmPerSecSq);

private:
    FastAccelStepper* _servo = nullptr;
    static FastAccelStepperEngine _engine;

    volatile MotorState _state = MOTOR_DISABLED;

    int32_t _minStep = 0;
    int32_t _maxStep = 0;
    uint32_t _maxStepPerSec = 0;
    uint32_t _maxStepAccel = 0;
    float _stepsPerMm = 0;
    float _keepout = 5.0;       // mm, from KEEPOUT_TRAVEL
    float _physicalTravel = 0;

    // Homing task management
    TaskHandle_t _homingTask = nullptr;
    void (*_homingCallback)(bool) = nullptr;

    // Homing abort flag - checked in homing loops, set by disable()
    volatile bool _abortHoming = false;

    // Pattern (speed mode) state
    TaskHandle_t _patternTask = nullptr;
    volatile bool _patternActive = false;
    volatile int32_t _patternMin = 0;
    volatile int32_t _patternMax = 0;
    volatile float _patternSpeed = 0;  // strokes per minute
    volatile uint32_t _patternGen = 0; // bumped on speed change to interrupt mid-stroke
    void _runPatternTask();
    static void patternTaskWrapper(void* params);

    // Homing parameters stored for task access
    uint8_t _homingEndstopPin = 0;
    bool _homingActiveLow = false;
    bool _homingToBack = true;
    float _homingSpeed = 0;
    uint8_t _homingCurrentPin = 0;
    float _homingThreshold = 0;
    float _homingMaxTravel = 0;

    // Helper for ADC averaging (returns percentage 0-100)
    float _getAnalogAveragePercent(int pin, int samples);

    // Internal homing task implementations
    void _runEndstopHomingTask();
    void _runSensorlessHomingTask();

    // Static wrapper functions for FreeRTOS task entry points
    static void endstopHomingTaskWrapper(void* params);
    static void sensorlessHomingTaskWrapper(void* params);
};

#endif // MOTOR_CONTROLLER_H
