# Code Review: `attempted-motion-improvement` branch

## Dead Code & Dead Definitions

1. **`config.h:11` - `SERVO_ALARM`** is defined as pin 26 (same as `SERVO_ENABLE`) and never referenced anywhere. Dead.

2. **`config.h:14` - `RGB_LED`** is defined and never used anywhere. Dead.

3. **`config.h:53` - `PHYSICAL_TRAVEL`** - Only used by `MAX_DEPTH`. Comment says "gets overwritten in StrokeEngine by homing" but StrokeEngine was removed from this branch.

4. **`config.h:55` - `MAX_DEPTH`** is defined but never referenced in any source file. Dead. Its dependency on `PHYSICAL_TRAVEL` and `KEEPOUT_TRAVEL` makes both of those dead too (since nothing else uses `PHYSICAL_TRAVEL`, and `KEEPOUT_TRAVEL` is used only by `MAX_DEPTH` - MotorController hardcodes `_keepout = 5.0` instead of using the constant).

5. **`utils/analog.h:6-9` - `SampleOnPin` struct** is defined and never used. Dead.

6. **`main.cpp:40` - `//StaticJsonDocument<1024> doc;`** - Commented-out line. Kill it.

7. **`main.cpp:264-265` - `getPatternList` action handler** is an empty `else if` block that does nothing. Either implement it or remove it.

8. **`main.cpp:334-336` - PWM setup code** (`ledcSetup`, `ledcAttachPin`, `ledcWrite(0, 0)`) - `ledcWrite` is only ever called once with value 0, and nothing else references the PWM channel. More importantly, **`PWM` is pin 21 which is the same as `displayData`/I2C SDA**. This is a pin conflict. This whole block is dead code that's potentially messing with your display.

9. **`StreamingController.cpp:195,200` - `cmdCount`** is incremented but never read. Dead variable.

10. **`platformio.ini:19` - `CircularBuffer` dependency** - Not included or referenced anywhere in the source. Dead dependency eating flash space.

11. **`BLEManager.h:23` - `settingsCharacteristic`** is declared `extern` but never defined in the .cpp. Dead/stale declaration.

12. **`BLEManager.h:25` - `softwareVersionCharacteristic`** is declared `extern` but the .cpp defines `softwareAPIVersionCharacteristic` and `softwareFirmwareVersionCharacteristic` instead. Stale declaration from a refactor that got the header out of sync.

## Bugs / Likely Bugs

13. **`main.cpp:28` - `long rebootInMillis`** is `long` (signed 32-bit on ESP32) but it's compared with `millis()` which returns `unsigned long`. After ~25 days of uptime, `millis() + 3000` overflows differently for signed vs unsigned. Should be `unsigned long`.

14. **`main.cpp:445-449` - WebSocket telemetry send** is not guarded by `#if COMPILE_WEBSOCKET`. If you ever set `COMPILE_WEBSOCKET false`, this won't compile because `WebsocketManager` won't be included.

15. **`BLEManager.cpp:105-106`** - `setMinPreferred` is called twice:
    ```cpp
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    ```
    The second call overwrites the first. The original intent was likely `setMinPreferred(0x06)` and `setMaxPreferred(0x12)` (or vice versa). As-is, only `0x12` takes effect.

16. **`MotorController.h:65` - `_keepout = 5.0` hardcoded** instead of using `KEEPOUT_TRAVEL` from config.h. If someone changes the config constant, MotorController silently disagrees.

17. **`MotorController.cpp:368`** - Endstop homing uses `_physicalTravel` for step limit calculation, but `_physicalTravel` is only set by `homeManual`, `homeSensorless`, or `setPhysicalTravel`. If endstop homing is invoked before any of those, `_physicalTravel` is 0, making `_minStep >= _maxStep`, and homing "succeeds" the endstop detection but then fails at the validation on line 373 with no useful feedback about why.

## Repetitive / Duplicated Code

18. **"Recalculate and apply stroke limits" pattern** appears 4 times in `processCommand`:
    - Lines 157-160 (`startStreaming`)
    - Lines 214-219 (`setStroke`)
    - Lines 224-227 (`setDepth`)
    - Lines 274-277 (`setPattern`)

    All do `getStrokeLimits(&strokeMin, &strokeMax)` then some combo of `streamingController.setStrokeLimits` / `Motor.setPatternLimits`. This should be a helper like `applyStrokeLimits()`.

19. **"Send JSON to all active transports" pattern** appears in:
    - `sendHomingResponse()` (lines 82-91)
    - `version` action (lines 252-262)
    - Telemetry block (lines 445-449, partially)

    Each repeats the `#if COMPILE_WEBSOCKET / #if COMPILE_BLUETOOTH` dance. A `sendToClients(const String& json)` helper would eliminate this.

20. **`MotorController::_getAnalogAveragePercent`** (line 163) is functionally identical to the free function `getAnalogAveragePercent` in `utils/analog.h`. Minor difference: the MotorController version uses `float sum` while analog.h uses `int sum`. One of them should go.

21. **Abort-homing boilerplate in `_runSensorlessHomingTask`** - There are **8** blocks that all do some variation of:
    ```cpp
    if (_abortHoming) {
        _servo->forceStop();
        _servo->disableOutputs();
        _state = MOTOR_DISABLED;
        if (_homingCallback) _homingCallback(false);
        return;
    }
    ```
    This is begging for either a `goto cleanup` pattern or a helper method.

22. **`processQueue()` is identical** in BLEManager and SerialManager - both drain a queue and call `msgReceivedCallback(String(buffer))`. Same logic, same shape, copy-pasted.

## Style / Consistency Issues

23. **Trailing semicolons on function/namespace bodies** - `main.cpp:286`, `main.cpp:293`, `main.cpp:357`, `main.cpp:455`, `BLEManager.cpp:30,35,41,108`, `SerialManager.cpp:81`. These are legal but unconventional and make it look like the functions are inside a struct/class.

24. **`boolean` vs `bool`** - `main.cpp:25-26` and `main.cpp:148` use Arduino's `boolean` typedef while everything else uses standard `bool`. Pick one (prefer `bool`).

25. **`WebsocketManager.h` declares `handleRoot` and `handle404` twice** (lines 22-23 and lines 43-47). Redundant.

26. **Hand-rolled JSON in the telemetry block** (`main.cpp:435-442`) builds a JSON object with `snprintf` while everything else uses ArduinoJson. Fragile and inconsistent.

27. **`STEP_PER_MM` macro** (`config.h:49`) lacks outer parentheses: `STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH)`. Should be `(STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH))`. Happens to not bite you in any current usage, but it's a time bomb.

## Misleading Comments

28. **`WebsocketManager.cpp:156-157`** - Comment says "passing messages around. If we receive something, we send it to all other clients" but the code calls `msgReceivedCallback`, not a broadcast. Inherited from the example code this was based on.

29. **`WebsocketManager.cpp:167`** - Comment says "The following HTML code will present the chat interface" but the handler returns plain text. Same provenance.

30. **`config.h:53`** - Comment says "gets overwritten in StrokeEngine by homing" but StrokeEngine was removed in this branch.

## Minor / Cleanup

31. **`display.cpp:120-127` - `showConnected`** uses `display.print()` without setting cursor position, while all other `show*` methods use `drawStr` with explicit coordinates. This probably renders at position (0,0) overlapping the icons.

32. **`display.cpp:132` - `showRunning`** explicitly calls `setFont(u8g2_font_cu12_tr)` but no other `show*` method does, suggesting either this is unnecessary (font never changes) or the other methods are missing it.

## Summary

The big wins for maintainability would be:
- **Kill the ~12 dead definitions/variables** (items 1-12)
- **Extract a `sendToClients()` helper** to eliminate the duplicated transport dispatch pattern
- **Extract an `applyStrokeLimits()` helper** in main.cpp
- **Fix the actual bugs**: PWM pin conflict, `rebootInMillis` type, missing `#if COMPILE_WEBSOCKET` guard, BLE `setMinPreferred` double-call, stale BLE header declarations
- **Use `KEEPOUT_TRAVEL`** from config instead of hardcoding 5.0 in MotorController
