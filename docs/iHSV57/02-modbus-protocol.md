# Modbus RTU Communication Protocol

## Physical Layer

The iHSV57 uses RS232 serial (NOT RS485, NOT TTL UART).

### Default Serial Settings (V6 firmware)

| Parameter | Value |
|-----------|-------|
| Baud rate | 57600 bps |
| Data bits | 8 |
| Parity | Even |
| Stop bits | 1 |
| Slave address | 1 |
| Response delay | 0 ms |

### Configurable Serial Parameters

**P00-21 - RS232 Baud Rate Selection** (for PC software connection)

| Value | Baud Rate |
|-------|-----------|
| 0 | 9600 |
| 1 | 19200 |
| 2 | 57600 (default) |
| 3 | 115200 |

**P00-24 - Modbus Baud Rate** (for Modbus communication)

| Value | Baud Rate |
|-------|-----------|
| 0 | 2400 |
| 1 | 4800 |
| 2 | 9600 |
| 3 | 19200 |
| 4 | 38400 |
| 5 | 57600 |
| 6 | 115200 |
| 7 | 25600 (default = 7) |

**NOTE**: P00-21 and P00-24 appear to be separate baud rate settings. P00-21 controls
the RS232 port baud rate for the PC software, P00-24 controls the Modbus baud rate.
In practice, they may need to match. The factory default for P00-24 is 7 (25600),
but most servos ship configured at 57600. Verify with your specific unit.

**P00-25 - Parity (Check Way)**

| Value | Setting |
|-------|---------|
| 0 | No parity, 2 stop bits |
| 1 | Even parity, 1 stop bit (default) |
| 2 | Odd parity, 1 stop bit |
| 3 | No parity, 1 stop bit |

**P00-23 - Slave Address**: 0-255, default 1

**P00-26 - Communication Response Delay**: 0-100 ms, default 0

All communication parameters require stop setting (servo disabled) and re-power to take effect.

## Protocol: Modbus RTU

Standard Modbus RTU with CRC-16/MODBUS.

### Supported Function Codes

| Code | Name | Use |
|------|------|-----|
| 0x03 | Read Holding Register(s) | Read one or more 16-bit registers |
| 0x06 | Write Single Register | Write one 16-bit register |

### Frame Formats

All multi-byte values are **big-endian** except CRC which is **little-endian**.

#### Read Request (8 bytes)

```
[SlaveAddr:1] [0x03] [RegAddr_Hi:1] [RegAddr_Lo:1] [Count_Hi:1] [Count_Lo:1] [CRC_Lo:1] [CRC_Hi:1]
```

Example - Read register 0x0080:
```
TX: 01 03 00 80 00 01 85 D2
     ^  ^  ^^^^^  ^^^^^  ^^^^^
     |  |  addr   count  CRC16 (little-endian)
     |  func 0x03 (read)
     slave addr 1
```

#### Read Response (7 bytes for single register)

```
[SlaveAddr:1] [0x03] [ByteCount:1] [Value_Hi:1] [Value_Lo:1] [CRC_Lo:1] [CRC_Hi:1]
```

Example - Response with value 0x0012:
```
RX: 01 03 02 00 12 XX XX
     ^  ^  ^  ^^^^^  ^^^^^
     |  |  |  value  CRC16
     |  |  byte count (2 = one register)
     |  func echo
     slave addr
```

#### Write Request (8 bytes)

```
[SlaveAddr:1] [0x06] [RegAddr_Hi:1] [RegAddr_Lo:1] [Value_Hi:1] [Value_Lo:1] [CRC_Lo:1] [CRC_Hi:1]
```

Example - Write 0x0000 to register 0x0006:
```
TX: 01 06 00 06 00 00 69 CB
```

#### Write Response (8 bytes - echo of request)

The servo echoes back the exact same frame on success:
```
RX: 01 06 00 06 00 00 69 CB
```

### CRC-16/MODBUS

- Polynomial: 0xA001 (reflected)
- Initial value: 0xFFFF
- Appended **little-endian** (low byte first)
- Standard implementation - lookup table in `go-servoc/src/crc.go`

Test vectors:
```
Data: 01 03 00 85 00 01  ->  CRC: 0xE395  ->  bytes: 95 E3
Data: 01 06 00 0A 00 00  ->  CRC: 0xC8A9  ->  bytes: A9 C8
Data: 01 06 00 06 00 00  ->  CRC: 0xCB69  ->  bytes: 69 CB
```

### Connection Handshake

To verify the servo is connected and responding, read register **0x0080**:
```
TX: 01 03 00 80 00 01 [CRC]
```

Expected response value: **0x0012**

If you get a different value or no response, check wiring, baud rate, and parity settings.

### Multi-Register Reads

Function code 0x03 supports reading multiple consecutive registers by setting count > 1.
The iHSV-Servo-Tool uses this for efficient monitoring - reading 4-8 registers in a single
request to reduce time skew between channels.

Response for N registers:
```
[SlaveAddr:1] [0x03] [ByteCount:1=N*2] [Value1_Hi:1] [Value1_Lo:1] ... [ValueN_Hi] [ValueN_Lo] [CRC:2]
```

### Timing Considerations

At 57600 baud, 8E1:
- One byte = 11 bits (start + 8 data + parity + stop) = ~191 us
- 8-byte request frame = ~1.5 ms
- 7-byte response frame = ~1.3 ms
- Minimum inter-frame gap (Modbus spec): 3.5 character times = ~0.67 ms
- **Theoretical max throughput**: ~300 request/response pairs per second
- **Practical throughput**: ~100-200 Hz depending on response delay

### V5 vs V6 Serial Defaults

| Parameter | V5 | V6 |
|-----------|----|----|
| Baud rate | 57600 | 57600 |
| Parity | None (8N1) | Even (8E1) |
| Register map | 0x0006-0x00A1 range | Different addresses |
