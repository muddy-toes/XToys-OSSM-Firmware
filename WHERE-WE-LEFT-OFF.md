# Where We Left Off - 2026-03-02

## Task Summary

Fixed sensorless homing on OSSM with iHSV57 servo driver. Two problems: (1) homing itself needed tuning for the iHSV57's characteristics, and (2) XToys was reporting "Homing failed" even when homing succeeded due to a BLE notification race condition. Also cleaned all non-JSON serial output so the serial line can be consumed by a JSON parser.

## Changes Made (committed as `109d1a2`)

| File | Change |
|------|--------|
| `src/MotorController.cpp` | Sensorless homing overhaul: removed MIN_TRAVEL_MM blind spot (caused iHSV57 alarm before detection), baseline from 1000 ADC samples read before disable/enable cycle, fast-traverse optimization in phase 2 (100mm/s through known-safe territory, slow 10mm before unknown), `!isRunning()` fallback for driver alarm, `forceStopAndNewPosition` in `disable()`, removed all non-JSON serial prints |
| `src/BLEManager.cpp` | Fixed race condition: `setValue()+notify()` replaced with `notify(data, length, true)` atomic overload. XToys writes volume updates to the same BLE characteristic every ~100ms, which overwrote the notification payload between the two calls. Added `deviceConnected` guard. Removed non-JSON serial prints. |
| `src/main.cpp` | Added `pendingHomingResult` volatile flag so homing notifications are sent from main loop context (BLE notifications from FreeRTOS tasks are unreliable). Split `homingNotification()` into callback (sets flag) and `sendHomingResponse()` (main loop sends BLE). Lowered sensorless current threshold from 1.5 to 1.0 for iHSV57 sensitivity. |
| `src/config.h` | `SENSORLESS_HOMING_SPEED` 25 -> 35 mm/s (XToys has 20s timeout; 25mm/s exceeded it for >250mm travel) |
| `src/StreamingController.cpp` | Removed diagnostic `[stream]` logging and force-delete warning |
| `src/WebsocketManager.cpp` | Removed non-JSON serial prints (server startup, new client) |

## Current State

- **Complete**: All changes committed, build succeeds, firmware_merged.bin generated
- **Tested on device**: Sensorless homing works, XToys reports success (tested twice)
- **Serial line is clean**: Only JSON goes over serial now (telemetry every 200ms + command responses)
- Branch: `attempted-motion-improvement`, 1 commit ahead of origin

## Key Decisions

1. **BLE notify(data, length) overload** - The critical fix. XToys constantly writes volume updates to the control characteristic. The old `setValue()`+`notify(true)` two-step was racy because NimBLE's higher-priority BLE task could preempt between the calls. The `notify(data, length, true)` overload sends the specified bytes directly without reading the characteristic's current value.

2. **Main-loop BLE notification** - BLE notifications sent from FreeRTOS homing task context didn't reliably deliver. Using a volatile flag (`pendingHomingResult`) to signal the main loop to send it. The main loop polls this flag and calls `sendHomingResponse()`.

3. **No MIN_TRAVEL_MM blind spot** - The old code waited 10mm of travel before checking current, but the iHSV57 servo would alarm before the firmware could detect the stall. Old StrokeEngine detected immediately. Matches that behavior now.

4. **Baseline before disable/enable** - Read 1000 ADC samples for baseline BEFORE the motor disable/enable cycle, matching old StrokeEngine order. The disable cycle changes current draw.

5. **Fast-traverse in phase 2** - After finding first wall in phase 1, we know how far we traveled. Phase 2 fast-traverses at 100mm/s (not max speed) through known-safe territory, then slows to 35mm/s detection speed 10mm before unknown territory.

6. **Current threshold 1.0** (was 1.5) - The iHSV57 is a closed-loop servo, not a stepper. Its current signature differs; 1.0 works better for stall detection.

7. **All serial output is JSON** - No more plain-text debug prints on serial. Consumers (XToys serial mode, SerialManager) can parse the stream without choking on non-JSON.

## Previous Session Issues (resolved)

- **Streaming crash at ~3:47 in funscript**: Fixed in prior commits by (a) passing `JsonDocument&` by reference instead of by value (heap fragmentation), and (b) using fixed high acceleration in streaming mode instead of per-move formula that assumed motor starts from rest.
- **Pattern mode too slow**: Fixed in prior commit by replacing broken accel formula.

## Next Steps

- Push to origin if desired (`git push`)
- Test streaming mode (funscript playback past the 3:47 mark) to confirm crash fix holds
- Consider whether `"heap"` field in periodic JSON telemetry is worth keeping long-term (currently included, useful for monitoring)

## Context to Re-read

1. `src/MotorController.cpp` lines 410-640 - Sensorless homing implementation with fast-traverse
2. `src/BLEManager.cpp` lines 45-54 - The atomic notify fix (this was the root cause of "Homing failed")
3. `src/main.cpp` lines 30-113 - pendingHomingResult mechanism and sendHomingResponse/homingNotification split
4. `src/StreamingController.cpp` lines 193-243 - Execute-and-wait tick task (the streaming model)

## XToys Protocol Reference

Deobfuscated XToys OSSM JS is at `/home/skm/xtoys-js/3318-readable.js`. Key findings:
- `startHoming()` (line 573): Sets 20-second timeout, sends home command
- `processMessage()` (line 511): Expects `[{"action":"home","success":true}]`
- XToys writes volume updates to the BLE control characteristic every ~100ms
- Deobfuscation script: `/home/skm/xtoys-js/deobfuscate.js`
