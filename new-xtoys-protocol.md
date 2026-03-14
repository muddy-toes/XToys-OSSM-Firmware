# OSSM BLE Protocol Specification

**Version:** 1.0 (Implicit)
**Status:** Implementation Reference
**Last Updated:** 2026-02-07

## Overview

The OSSM (Open Source Sex Machine) implements a custom Bluetooth Low Energy (BLE) protocol for device control. This document provides a complete protocol specification for implementers building compatible devices or controllers.

### Design Philosophy

- **Text-based commands**: Human-readable command format for easy debugging
- **JSON state responses**: Structured data for easy parsing
- **UUID-based organization**: Characteristics grouped by function into UUID ranges
- **Validation-first**: All commands validated via regex before processing

### Minimum Viable Implementation

To create a basic OSSM-compatible device, you MUST implement:

1. BLE service with UUID `522b443a-4f53-534d-0001-420badbabe69`
2. Command characteristic (UUID ending in `1000`) - WRITE, READ
3. State characteristic (UUID ending in `2000`) - READ, NOTIFY
4. Command validation using the protocol regex
5. JSON state response format

Optional but recommended:
- Pattern characteristics (`3000`, `3010`)
- GPIO control (`4000`)
- Speed knob configuration (`1010`)
- FTS emulation mode

## Quick Start Example

```python
# Connect to OSSM device
device = await connect_ble("522b443a-4f53-534d-0001-420badbabe69")

# Get command characteristic
cmd_char = device.get_characteristic("522b443a-4f53-534d-1000-420badbabe69")

# Send a command (set speed to 75%)
await cmd_char.write(b"set:speed:75")

# Subscribe to state updates
state_char = device.get_characteristic("522b443a-4f53-534d-2000-420badbabe69")
await state_char.start_notify(handle_state_update)

def handle_state_update(sender, data):
    state = json.loads(data.decode())
    print(f"Speed: {state['speed']}, Stroke: {state['stroke']}")
```

## Version & Compatibility

**Important:** The current protocol implementation has NO version negotiation mechanism. The firmware does NOT expose:
- Protocol version number
- Firmware version via BLE
- Standard BLE Device Information characteristics (Model Number, Firmware Revision)

**Recommendation:** Implement feature detection by checking for characteristic existence rather than version checking.

This document describes the protocol as implemented in OSSM firmware as of early 2025. Future versions may extend the protocol using reserved UUID ranges.

---

## BLE Service Architecture

### Primary Service

**Service UUID:** `522b443a-4f53-534d-0001-420badbabe69`

All characteristics use the same UUID prefix with different ranges for organization.

### UUID Namespace Organization

The protocol uses a structured UUID namespace with ranges for different functions:

| Range | Purpose | Description |
|-------|---------|-------------|
| `0002-0FFF` | System | Reserved, not currently used |
| `1000-1FFF` | Commands & Config | Writable characteristics that change device behavior |
| `2000-2FFF` | State | Read/notify characteristics for device state |
| `3000-3FFF` | Patterns | Pattern information and selection |
| `4000-4FFF` | GPIO | GPIO control and status |

### Characteristics Summary

| UUID (Last 4 digits) | Range | Properties | Purpose |
|---------------------|-------|------------|---------|
| `1000` | Command | WRITE, READ | Main command input |
| `1010` | Command | WRITE, READ | Speed knob configuration |
| `2000` | State | READ, NOTIFY | Current state JSON |
| `3000` | Pattern | READ | List of available patterns |
| `3010` | Pattern | WRITE, READ | Get pattern description |
| `4000` | GPIO | WRITE, READ | GPIO pin control |

### Standard BLE Services

The device also implements:

**Device Information Service** (UUID: `180A`)
- Manufacturer Name (`2A29`): "Research And Desire"
- System ID (`2A23`): `88:1A:14:FF:FE:34:29:63`

Note: Model Number and Firmware Revision characteristics are NOT implemented.

---

## Command Protocol

### Command Characteristic

**UUID:** `522b443a-4f53-534d-1000-420badbabe69`
**Properties:** WRITE, READ
**Format:** UTF-8 text string

### Command Validation

All commands MUST match this regex pattern:

```regex
go:(simplePenetration|strokeEngine|streaming|menu)|set:(speed|stroke|depth|sensation|pattern):\d+|stream:\d+:\d+
```

Invalid commands receive a `fail:<command>` response.

### Command Types

#### 1. Navigation Commands (go:)

Navigate between device modes/states.

**Format:** `go:<mode>`

| Command | Effect |
|---------|--------|
| `go:strokeEngine` | Switch to Stroke Engine mode (pattern-based motion) |
| `go:simplePenetration` | Switch to Simple Penetration mode (basic in/out) |
| `go:streaming` | Switch to Streaming mode (real-time position control) |
| `go:menu` | Return to main menu |

**Example:**
```
> go:strokeEngine
< ok:go:strokeEngine
< {"state":"strokeEngine.idle","speed":50,"stroke":50,"sensation":50,"depth":50,"pattern":0}
```

#### 2. Configuration Commands (set:)

Set device parameters (0-100 scale).

**Format:** `set:<parameter>:<value>`

**Parameters:**

| Parameter | Range | Description |
|-----------|-------|-------------|
| `speed` | 0-100 | Motor speed percentage |
| `stroke` | 0-100 | Stroke length percentage |
| `depth` | 0-100 | Penetration depth percentage |
| `sensation` | 0-100 | Sensation intensity level |
| `pattern` | 0-100 | Pattern index (value % 7 for 7 patterns) |

**Validation:**
- Value MUST be integer 0-100
- Values outside range are rejected with `fail:` response
- Pattern value is wrapped using modulo 7

**Examples:**
```
> set:speed:75
< ok:set:speed:75
< {"state":"strokeEngine.idle","speed":75,"stroke":50,"sensation":50,"depth":50,"pattern":0}

> set:pattern:5
< ok:set:pattern:5
< {"state":"strokeEngine.idle","speed":75,"stroke":50,"sensation":50,"depth":50,"pattern":5}
```

#### 3. Streaming Commands (stream:)

Real-time position control for synchronized motion.

**Format:** `stream:<position>:<time>`

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `position` | integer | 0-100 | Target position percentage (0=retracted, 100=extended) |
| `time` | integer | 0+ | Milliseconds to reach position |

**Position Scaling:**
- External protocol: 0-100 (percentage)
- Internal representation: 0-180 (scaled automatically)

**Examples:**
```
> stream:50:500
< ok:stream:50:500
(Device moves to 50% position over 500ms)

> stream:100:1000
< ok:stream:100:1000
(Device extends to 100% position over 1 second)

> stream:0:250
< ok:stream:0:250
(Device retracts to 0% position over 250ms)
```

**Use Case:** Streaming mode is designed for synchronized playback of motion scripts where position and timing are pre-defined.

### Command Response Format

**Success Response:**
```
ok:<command_string>
```

**Failure Response:**
```
fail:<command_string>
```

After any command, the device will also send a state notification with the updated JSON state.

---

## State Protocol

### State Characteristic

**UUID:** `522b443a-4f53-534d-2000-420badbabe69`
**Properties:** READ, NOTIFY
**Format:** UTF-8 JSON string or text response

### Boot Message

When the device boots or resets:
```
ok:boot
```

### State JSON Format

```json
{
  "state": "<state_name>",
  "speed": 0-100,
  "stroke": 0-100,
  "sensation": 0-100,
  "depth": 0-100,
  "pattern": 0-6
}
```

**Fields:**

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `state` | string | - | Current state machine state (see State Names) |
| `speed` | integer | 0-100 | Current speed setting |
| `stroke` | integer | 0-100 | Current stroke length |
| `sensation` | integer | 0-100 | Current sensation level |
| `depth` | integer | 0-100 | Current depth setting |
| `pattern` | integer | 0-6 | Current pattern index (7 patterns total) |

### State Names

The `state` field reflects the device's state machine state. Common states include:

- `menu.idle` - Device at main menu
- `strokeEngine.idle` - Stroke Engine mode, not moving
- `simplePenetration.idle` - Simple Penetration mode, not moving
- `streaming.idle` - Streaming mode, not moving

Additional states exist for active motion, transitions, and error conditions.

### Notification Behavior

State notifications are sent:
- **On change:** Whenever any parameter changes
- **Periodic:** Every ~1 second (baseline heartbeat)
- **After commands:** Following command acknowledgment

**Implementation Note:** Controllers SHOULD subscribe to notifications rather than polling via READ.

---

## Speed Knob Configuration

### Speed Knob Characteristic

**UUID:** `522b443a-4f53-534d-1010-420badbabe69`
**Properties:** WRITE, READ
**Format:** UTF-8 text string

### Purpose

Toggles the physical speed knob behavior between:
- **Direct speed control** (true): Knob directly sets speed
- **Speed limit** (false): Knob sets maximum speed limit

### Command Format

**Write one of:**
- `true`, `false` (case-insensitive)
- `t`, `f` (case-insensitive)
- `1`, `0` (numeric)

**Response:**
The device echoes back the normalized value:
```
true
```
or
```
false
```

**Error Response:**
```
error:invalid_value
```

**Example:**
```
> true
< true
(Speed knob now controls direct speed)

> false
< false
(Speed knob now sets speed limit)
```

---

## Pattern System

### Pattern List Characteristic

**UUID:** `522b443a-4f53-534d-3000-420badbabe69`
**Properties:** READ
**Format:** UTF-8 JSON array

### Format

```json
[
  {"name": "Pattern Name 1", "index": 0},
  {"name": "Pattern Name 2", "index": 1},
  {"name": "Pattern Name 3", "index": 2},
  ...
]
```

**Fields:**
- `name`: Human-readable pattern name
- `index`: Pattern index for use with `set:pattern:N` command

### Pattern Data Characteristic

**UUID:** `522b443a-4f53-534d-3010-420badbabe69`
**Properties:** WRITE, READ
**Format:** Integer (write), UTF-8 text (read)

### Usage

1. **Write** pattern index (integer) to request description
2. **Read** to get pattern description string

**Example:**
```
> WRITE: 2
> READ: "A slow, teasing pattern with long pauses"
```

### Selecting Patterns

Use the `set:pattern:N` command (see Command Protocol section):
```
set:pattern:2
```

Pattern values wrap using modulo arithmetic, so `set:pattern:10` with 7 patterns selects pattern `10 % 7 = 3`.

---

## GPIO Control

### GPIO Characteristic

**UUID:** `522b443a-4f53-534d-4000-420badbabe69`
**Properties:** WRITE, READ
**Format:** UTF-8 text string

### Command Format

**Write:** `<pin>:<state>`

| Field | Valid Values | Description |
|-------|-------------|-------------|
| `pin` | 1-4 | GPIO pin number |
| `state` | `high`, `low`, `1`, `0` | Desired pin state (case-insensitive) |

**Example Commands:**
```
2:high
4:low
1:1
3:0
```

### Response Format

**Success:**
```
ok:<pin>:<state>
```

**Example:**
```
> 2:high
< ok:2:high
```

**Error Responses:**
```
error:invalid_format
error:pin_out_of_range
```

### Capability Discovery

**Read** the characteristic to get available GPIO information (implementation-specific).

### Pin Mapping

Pins 1-4 map to the device's GPIO outputs. Physical pin mapping is hardware-specific.

---

## Streaming Mode

### Overview

Streaming mode enables real-time position control for synchronized motion playback (e.g., video sync, script playback).

### Activation

```
go:streaming
```

### Position Commands

Use `stream:` commands as described in Command Protocol:

```
stream:<position>:<time>
```

### Timing Behavior

- Device interpolates motion from current position to target position
- Motion completes in the specified time (milliseconds)
- Commands can be queued for smooth continuous motion
- Time value of 0 is valid (immediate motion)

### Scaling Details

**Important:** Position values use different scales at protocol vs. internal level:

- **Protocol (external):** 0-100 (percentage scale)
- **Internal (device):** 0-180 (scaled automatically by device)

Controllers MUST use 0-100 scale in commands. The device handles internal scaling.

### Example Sequence

```
go:streaming                    # Enter streaming mode
stream:0:0                      # Move to start position immediately
stream:50:1000                  # Move to 50% over 1 second
stream:100:500                  # Move to 100% over 0.5 seconds
stream:20:800                   # Move to 20% over 0.8 seconds
```

---

## FTS Emulation Mode (Optional)

### Overview

When compiled with `PRETEND_TO_BE_FLESHY_THRUST_SYNC` flag, the device can emulate a Fleshy Thrust Sync (FTS) device for backward compatibility with FTS controllers.

**Note:** This is an optional compile-time feature. Devices may not support it.

### FTS BLE Service

**Service UUID:** `0000ffe0-0000-1000-8000-00805f9b34fb`
**Characteristic UUID:** `0000ffe1-0000-1000-8000-00805f9b34fb`
**Properties:** WRITE

### FTS Binary Protocol Format

**Packet Size:** 3 bytes

```
[Position (uint8)] [Time High (uint8)] [Time Low (uint8)]
```

| Byte | Type | Range | Description |
|------|------|-------|-------------|
| 0 | uint8 | 0-180 | Target position (0=retracted, 180=extended) |
| 1 | uint8 | - | Time MSB (most significant byte) |
| 2 | uint8 | - | Time LSB (least significant byte) |

**Time Encoding:** uint16 big-endian (MSB first)
- Bytes 1-2 encode milliseconds to reach position
- Combine as: `time = (byte1 << 8) | byte2`

**Position Scale:**
- FTS uses 0-180 scale (not 0-100 like OSSM stream commands)
- 0 = fully retracted
- 180 = fully extended

### Example FTS Packet

Move to position 90 over 500ms:
```
Byte 0: 90       (position)
Byte 1: 0x01     (500 >> 8 = 1)
Byte 2: 0xF4     (500 & 0xFF = 244)

Hex: 5A 01 F4
```

### Python Example

```python
import struct

def create_fts_packet(position, time_ms):
    """Create FTS packet: position (0-180), time in milliseconds"""
    time_high = (time_ms >> 8) & 0xFF
    time_low = time_ms & 0xFF
    return bytes([position, time_high, time_low])

# Move to position 90 over 500ms
packet = create_fts_packet(90, 500)
await fts_char.write(packet)
```

### FTS vs. OSSM Stream Commands

| Feature | FTS Mode | OSSM Stream |
|---------|----------|-------------|
| Protocol | Binary | Text |
| Position Scale | 0-180 | 0-100 |
| Service UUID | `0000ffe0-...` | `522b443a-...` |
| Byte Order | Big-endian | N/A (text) |
| Validation | None (binary) | Regex (text) |

**Recommendation:** New implementations should use the native OSSM protocol rather than FTS emulation unless FTS compatibility is specifically required.

---

## Implementation Guidelines

### Connection Sequence

1. **Scan** for BLE devices advertising service UUID `522b443a-4f53-534d-0001-420badbabe69`
2. **Connect** to the device
3. **Discover services** and verify expected characteristics exist
4. **Subscribe** to state characteristic (`2000`) for notifications
5. **Send commands** via command characteristic (`1000`)
6. **Monitor state** updates via notifications

### Command Validation

**Before sending commands:**
- Validate format against regex pattern
- Ensure values are in valid ranges (0-100 for set commands)
- Use UTF-8 encoding

**After sending commands:**
- Wait for acknowledgment (`ok:` or `fail:`)
- Check state notification for updated values
- Handle `fail:` responses appropriately

### Error Handling

**Command Failures:**
- Device returns `fail:<command>` if validation fails
- State does NOT change on failed commands
- Controller should retry or alert user

**Connection Loss:**
- Device waits 1 second after disconnect
- Device ramps speed down to 0 over 2 seconds
- Controllers should reconnect and resend last known state

**Invalid State:**
- Some commands may be rejected based on device state (e.g., safety limits)
- Check state notifications for actual device behavior

### Timing Considerations

**State Notifications:**
- Expect updates every ~1 second baseline
- Additional updates on parameter changes
- Don't rely on exact 1Hz timing (use notifications, not polling)

**Command Response Time:**
- Acknowledgments typically arrive within 50-100ms
- State notifications follow shortly after
- Don't send rapid commands without waiting for ack (can overwhelm queue)

**Streaming Motion:**
- Commands specify time to reach position
- Device handles interpolation and motion planning
- Send next command before current motion completes for smooth playback

### Testing Recommendations

**Basic Functionality:**
1. Connect and verify all characteristics exist
2. Subscribe to state notifications
3. Send each command type and verify acknowledgment
4. Verify state updates reflect commands
5. Test invalid commands and verify `fail:` response

**Edge Cases:**
1. Send values at boundaries (0, 100)
2. Send invalid values (negative, >100)
3. Send malformed commands
4. Test rapid command sequences
5. Test connection loss and reconnection

**Streaming Mode:**
1. Verify smooth motion with sequential commands
2. Test rapid position changes
3. Test extreme timing values (0ms, 5000ms)
4. Verify position scaling (0-100 external)

---

## Limitations & Caveats

### Protocol Versioning

**No Version Negotiation**
- Protocol has no version characteristic
- No handshake or capability exchange
- Controllers cannot query protocol version
- Firmware version not exposed via BLE

**Workaround:**
- Use feature detection (check characteristic existence)
- Assume baseline feature set (command, state characteristics)
- Gracefully handle missing optional characteristics

### Device Information

**Missing Standard Characteristics:**
- Model Number (2A24) - NOT implemented
- Hardware Revision (2A25) - NOT implemented
- Firmware Revision (2A26) - NOT implemented

Only Manufacturer Name and System ID are provided.

### Command Validation

**Regex Validation:**
- Commands MUST match exact regex pattern
- No partial matching or fuzzy parsing
- Whitespace not allowed in commands
- Case-sensitive (use lowercase)

**Pattern Wrapping:**
- Pattern index uses modulo 7
- `set:pattern:10` selects pattern 3
- This is intentional behavior, not a bug

### State Machine Behavior

**State-Dependent Commands:**
- Some commands may be rejected based on current state
- Safety limits may prevent certain motions
- Device may require homing before first motion
- State field indicates current mode/state

### Disconnection Behavior

**On Disconnect:**
1. Device waits 1 second (configurable in firmware)
2. Speed ramps down to 0 over 2 seconds
3. Device returns to safe/idle state

Controllers should implement graceful disconnect with explicit stop commands when possible.

### Performance Characteristics

**MTU:** Default BLE MTU (typically 20-256 bytes)
- Commands are short text strings (under 30 bytes typically)
- State JSON under 200 bytes
- MTU negotiation not required but may improve performance

**Throughput:**
- Command rate limited by queue processing
- Don't exceed ~10-20 commands/second
- State notifications sent at ~1Hz baseline

---

## Appendices

### Appendix A: Complete UUID Reference

| Characteristic | Full UUID |
|---------------|-----------|
| Service | `522b443a-4f53-534d-0001-420badbabe69` |
| Command | `522b443a-4f53-534d-1000-420badbabe69` |
| Speed Knob Config | `522b443a-4f53-534d-1010-420badbabe69` |
| State | `522b443a-4f53-534d-2000-420badbabe69` |
| Pattern List | `522b443a-4f53-534d-3000-420badbabe69` |
| Pattern Data | `522b443a-4f53-534d-3010-420badbabe69` |
| GPIO | `522b443a-4f53-534d-4000-420badbabe69` |
| FTS Service (optional) | `0000ffe0-0000-1000-8000-00805f9b34fb` |
| FTS Characteristic (optional) | `0000ffe1-0000-1000-8000-00805f9b34fb` |

### Appendix B: Complete Command Reference

| Command | Format | Values | Description |
|---------|--------|--------|-------------|
| Go to Stroke Engine | `go:strokeEngine` | - | Enter pattern-based mode |
| Go to Simple Penetration | `go:simplePenetration` | - | Enter simple in/out mode |
| Go to Streaming | `go:streaming` | - | Enter real-time control mode |
| Go to Menu | `go:menu` | - | Return to main menu |
| Set Speed | `set:speed:N` | 0-100 | Set motor speed % |
| Set Stroke | `set:stroke:N` | 0-100 | Set stroke length % |
| Set Depth | `set:depth:N` | 0-100 | Set depth % |
| Set Sensation | `set:sensation:N` | 0-100 | Set sensation level |
| Set Pattern | `set:pattern:N` | 0-100 | Set pattern (N % 7) |
| Stream Position | `stream:N:T` | N=0-100, T=0+ | Move to position N in T ms |

### Appendix C: Python Example (Bleak)

```python
import asyncio
import json
from bleak import BleakClient, BleakScanner

OSSM_SERVICE_UUID = "522b443a-4f53-534d-0001-420badbabe69"
COMMAND_UUID = "522b443a-4f53-534d-1000-420badbabe69"
STATE_UUID = "522b443a-4f53-534d-2000-420badbabe69"

async def main():
    # Find OSSM device
    print("Scanning for OSSM device...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: OSSM_SERVICE_UUID.lower() in ad.service_uuids
    )

    if not device:
        print("OSSM device not found!")
        return

    print(f"Found device: {device.name}")

    async with BleakClient(device) as client:
        print("Connected!")

        # Subscribe to state updates
        def state_callback(sender, data):
            try:
                state = json.loads(data.decode())
                print(f"State: {state}")
            except:
                print(f"Boot message: {data.decode()}")

        await client.start_notify(STATE_UUID, state_callback)

        # Send some commands
        print("\nSending commands...")

        # Set speed to 50%
        await client.write_gatt_char(COMMAND_UUID, b"set:speed:50")
        await asyncio.sleep(1)

        # Set stroke to 75%
        await client.write_gatt_char(COMMAND_UUID, b"set:stroke:75")
        await asyncio.sleep(1)

        # Switch to stroke engine mode
        await client.write_gatt_char(COMMAND_UUID, b"go:strokeEngine")
        await asyncio.sleep(1)

        # Try streaming mode
        await client.write_gatt_char(COMMAND_UUID, b"go:streaming")
        await asyncio.sleep(1)

        # Stream some positions
        for pos in [0, 50, 100, 50, 0]:
            cmd = f"stream:{pos}:500".encode()
            await client.write_gatt_char(COMMAND_UUID, cmd)
            await asyncio.sleep(0.5)

        print("\nDone! Waiting for final state updates...")
        await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(main())
```

### Appendix D: Feature Detection Strategy

Since the protocol lacks version negotiation, implement feature detection:

```python
async def detect_features(client):
    """Detect available OSSM features by checking characteristic existence"""
    features = {
        'command': False,
        'state': False,
        'speed_knob_config': False,
        'patterns': False,
        'pattern_data': False,
        'gpio': False,
        'fts_emulation': False
    }

    try:
        # Check for OSSM service
        services = await client.get_services()
        ossm_service = services.get_service(OSSM_SERVICE_UUID)

        if ossm_service:
            # Check for each characteristic
            for char in ossm_service.characteristics:
                if char.uuid.endswith("1000"):
                    features['command'] = True
                elif char.uuid.endswith("1010"):
                    features['speed_knob_config'] = True
                elif char.uuid.endswith("2000"):
                    features['state'] = True
                elif char.uuid.endswith("3000"):
                    features['patterns'] = True
                elif char.uuid.endswith("3010"):
                    features['pattern_data'] = True
                elif char.uuid.endswith("4000"):
                    features['gpio'] = True

        # Check for FTS service
        FTS_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
        try:
            fts_service = services.get_service(FTS_SERVICE_UUID)
            if fts_service:
                features['fts_emulation'] = True
        except:
            pass

    except Exception as e:
        print(f"Error detecting features: {e}")

    return features
```

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-02-07 | Initial protocol documentation |

## References

- OSSM Firmware Source: `/home/skm/ossm/OSSM-hardware/Software/`
- OSSM Project: https://github.com/KinkyMakers/OSSM-hardware

## License

This protocol documentation is provided for interoperability purposes. The OSSM project is open source.

---

**End of OSSM BLE Protocol Specification**
