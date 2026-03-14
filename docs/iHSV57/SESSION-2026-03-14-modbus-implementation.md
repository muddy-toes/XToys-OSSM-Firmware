# Session Notes: 2026-03-14 - Modbus Implementation

## What Happened This Session

### Phase 1: Research Blitz

Deployed 16 research agents in parallel to gather everything available about the iHSV57 servo's serial communication capabilities. Over 1.2 million tokens of research across 800+ web searches and tool calls.

**Key discoveries:**
- The iHSV57 speaks **Modbus RTU over RS232** at 57600 baud, 8E1 (V6 firmware)
- Standard Modbus function codes 0x03 (read) and 0x06 (write)
- The OSSM Reference Board **already has an RS232 transceiver** (SP3232E) on UART2 (GPIO 16/17) with a 3-pin header - just needs a cable
- Complete V6 register map extracted: 185+ registers across 0x0000-0x0844
- V5 and V6 firmwares use completely different register addresses
- The servo can do far more than step/dir: speed control, position control, torque control, and real-time telemetry all over Modbus
- The ossm-rs project (Rust) has already proven direct Modbus motor control works in OSSM (with 57AIM motor)

**Repos cloned to docs/iHSV57-repos/ (not committed, ~112MB):**
- go-servoc (Go CLI, Apache 2.0) - CRC table used in our code
- iHSV-Servo-Tool (Python GUI, GPL v3) - definitive register map in iHSV_Properties.py (9450 lines)
- iHSV57-Arduino (Arduino, MIT) - proof of concept, only setTorque()
- JMC-Servo-Configuration (no license) - PDF manuals, JMC PC software
- JMC_servo_tuning (MIT) - parameter extraction, tuning guide with 485 parameters
- ossm-rs (Rust, Apache 2.0) - full Modbus motor control firmware for 57AIM

**14 reference docs written to docs/iHSV57/:**
Overview, Modbus protocol, register map, control modes, DIP switches, wiring, PID tuning, alarm codes, monitoring registers, software/tools, ESP32 integration notes, OSSM-specific notes, sources.

### Phase 2: Design Decisions

**What to build:** Telemetry reads + automatic compliance via the servo's built-in gain switching. No speed mode, no direct position control - those are future work.

**Why gain switching:** The servo has a P02-30 parameter that configures automatic switching between two gain sets based on torque. Mode 3 = switch to soft gains when torque exceeds a threshold. The servo's internal loop runs at 2kHz, far faster than anything we could do over Modbus. We just set the rules once and let it run.

**Why not adaptive threshold:** Initially built a Modbus feedback loop that read torque at 50Hz and adjusted the switching threshold every 200ms. Research revealed that every Modbus write likely goes to EEPROM on the iHSV57 (no RAM-only write mode, no save command). Frequent writes would burn out the EEPROM in hours. Switched to fixed threshold with read-before-write on startup.

**Why hand-rolled Modbus:** The eModbus library pulls in AsyncTCP and Ethernet as PlatformIO dependencies even when you only use RTU. The protocol is trivial (8-byte frames with CRC-16), so hand-rolling avoids the dependency bloat. Total implementation: ~200 lines of C++ for the full Modbus RTU master.

### Phase 3: Implementation

**ModbusManager.h** - Register addresses, compliance constants, telemetry struct, public API
**ModbusManager.cpp** - CRC-16/MODBUS, frame send/receive, register read/write, ensureRegister (read-before-write), telemetry round-robin, initServo configuration

**Integration in main.cpp:**
- setup(): Initialize UART2
- After homing (deferred to main loop via pendingHomingResult): initServo() configures gain switching
- loop(): One Modbus read per iteration, round-robin across speed/torque registers
- Telemetry output: rpm and torque included in periodic JSON debug output

**Code review caught:**
1. initServo() was called from homing FreeRTOS task - moved to main loop (same pattern as BLE notifications)
2. Blocking reads could stack 300ms in one loop - limited to one Modbus op per iteration
3. readRegisters buffer overflow potential - added bounds check
4. State flags needed volatile qualifier
5. Health check was redundant with telemetry reads - removed
6. Modbus error responses (bit 7 set on function code) weren't being logged - added

### Phase 4: Tuning on Hardware

**First test (50% gain reduction):**
- Soft gains: pos=20.0, spd=12.0, int=30.0ms
- Switching threshold: 800
- Result: Motor went completely limp under pushback, dropped backwards, skipped belt teeth. Power pulled.

**Torque readings observed:**
- Mid-stroke cruising: 50-80
- Endpoint reversals (inertia): ~400
- Pushback starts: 400-500
- Hard pushback: 600+

**Problem identified:** Threshold at 800 was below normal endpoint reversal torque (~400), so the motor was switching to soft gains on every single stroke reversal, not just on resistance.

**Second test (25% gain reduction, raised threshold):**
- Soft gains: pos=36.0, spd=20.0, int=15.0ms
- Switching threshold: 2000 (above normal reversal peaks)
- Hysteresis: 200
- Result: Bench test looks reasonable. Real-world testing pending.

### Phase 5: EEPROM Protection

**ensureRegister pattern:** Every register is read before writing. If the current value matches the desired value, the write is skipped entirely. On first boot this writes ~7 registers (the ones that differ from factory). On subsequent boots, zero writes because the values persist.

**Factory defaults for registers we change:**
- P02-01 (pos gain 2): factory 570, we set 360
- P02-13 (spd gain 2): factory 270, we set 200
- P02-14 (spd integral 2): factory 10000, we set 150
- P02-30 (gain switch mode): factory 0, we set 3
- P02-31 (switch level): factory 800, we set 2000
- P02-32 (hysteresis): factory 100, we set 200
- P02-33 (switch delay): factory 100, we keep 100 (skipped)

### Licensing

- CRC table from go-servoc (Apache 2.0) - attributed in code comment
- Register addresses are factual data from JMC manuals - not copyrightable
- docs/iHSV57/ files are original work
- docs/iHSV57-repos/ excluded from public repo (GPL v3 and unlicensed content)

## Commits on branch ihsv57-modbus

1. `c97765f` - Fix streaming speed, refactor homing, clean up codebase (includes docs/iHSV57/ creation)
2. `8935db9` - Add Modbus RTU servo telemetry and automatic compliance for iHSV57
3. `e432087` - Remove local-only path references from iHSV57 sources doc
4. `d6e18aa` - Add proper Apache 2.0 attribution for CRC table from go-servoc

## What Still Needs Testing

- [ ] Real-world test with actual use (not just bench pushing)
- [ ] Verify compliance triggers only on genuine resistance, not normal operation
- [ ] Verify motor never loses position authority in soft mode
- [ ] Extended run for stability
- [ ] Test with different speeds/patterns
- [ ] Check if the torque threshold (2000) is in the right ballpark
- [ ] Confirm EEPROM reads show correct values on power cycle (ensureRegister skips)
- [ ] Test with RS232 cable disconnected - verify graceful degradation
