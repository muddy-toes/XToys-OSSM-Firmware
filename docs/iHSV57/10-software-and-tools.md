# Software & Tools

## Official JMC Software

### JmcServoPcControl

- **Latest version**: 2.4.6
- **Download**: https://www.jmc-motor.com/download/list/17.html
- **Also in repo**: `docs/iHSV57-repos/JMC-Servo-Configuration/JmcServoPcControl2.4.6.rar`
- **Platform**: Windows (XP with .NET 4, Windows 7 x64, Windows 10 x64)
- **Must run as Administrator**

**Features:**
- Parameter read/write for all ~100 servo parameters
- Real-time motor state monitoring
- Built-in oscilloscope (position/velocity/current graphs)
- Auto-tuning via P01-02/P01-03
- Connection auto-scan (v2.4+: scans baud, parity, stop bits)

**Connection dialog settings:**
- Port: COM3-COM8 (do not use COM1-2 or above COM8)
- Baud rate: 57600
- Data bits: 8
- Parity: Even
- Stop bits: 1
- Slave address: 1
- Type: "一体化伺服" (Integrated servo) - but select "AC servo" / "General servo"
  for access to hidden parameters like P06-40

**Tip from MikesMachines**: Some important parameters (like P06-40 for >1000 RPM
in speed mode) are only visible when selecting "AC servo motor" instead of
"Integrated servo motor" in the software.

### Older Versions

- v1.7.6: In repo as `JMC1.7.6.zip`. Has issues on Windows 10/11.
- v2.4.4: In repo as `JMCServoPcControl2.4.4.rar`.

---

## Open Source Tools

### iHSV-Servo-Tool (Python GUI) - RECOMMENDED

- **Repo**: https://github.com/robert-budde/iHSV-Servo-Tool
- **Fork by theelims**: https://github.com/theelims/iHSV-Servo-Tool
- **Local**: `docs/iHSV57-repos/iHSV-Servo-Tool/`
- **Language**: Python 3 (PyQt5, PyQtGraph, PySerial, MinimalModbus, NumPy)
- **Platform**: Linux, macOS, Windows

**Features:**
- Real-time oscilloscope with configurable plots, colors, dual Y-axes
- ~100 Hz update rate via bulk Modbus register reads
- Read/write ALL known servo parameters via table UI
- Inline parameter editing (writes to servo on cell exit)
- Complete register map for V5 AND V6 firmware (`iHSV_Properties.py` - 9450 lines)

**Key file**: `iHSV_Properties.py` contains the definitive register address mapping
for every parameter on both firmware versions. This is the single most comprehensive
reference for iHSV57 Modbus registers.

**Dependencies:**
```bash
pip install pyqt5 pyqtgraph pyserial minimalmodbus numpy
```

**Note from theelims (StrokeEngine author)**: The fork by theelims suggests
cross-pollination between OSSM motion control and servo tuning knowledge.

---

### go-servoc (Go CLI)

- **Repo**: https://github.com/tcurdt/go-servoc
- **Local**: `docs/iHSV57-repos/go-servoc/`
- **Language**: Go
- **Platform**: macOS, Linux, Windows

**Features:**
- Command-line parameter upload from YAML config files
- Complete Modbus RTU protocol implementation with CRC
- Connection verification (register 0x0080 check)

**Limitation**: Write-only tool - uploads config but cannot read current servo state.

**Key files:**
- `src/protocol.go` - Modbus RTU framing
- `src/config_write.go` - Register address map (V5)
- `src/crc.go` - CRC-16/MODBUS lookup table
- `configs/sebastian.yaml` - Real-world tuning values

---

### iHSV57-Arduino (C++/Arduino)

- **Repo**: https://github.com/flaretek/iHSV57-Arduino
- **Local**: `docs/iHSV57-repos/iHSV57-Arduino/`
- **Language**: C++ (Arduino/PlatformIO)
- **Platform**: ESP32, ESP8266

Minimal proof-of-concept. Only implements `setTorque()` writing to register 0x01FE.
Uses eModbus (miq19/eModbus v1.6.0) for Modbus RTU transport. Useful as a starting
point for ESP32 Modbus integration but needs significant expansion.

---

### JMC-Servo-Configuration (Documentation)

- **Repo**: https://github.com/MikesMachines/JMC-Servo-Configuration
- **Local**: `docs/iHSV57-repos/JMC-Servo-Configuration/`

Information aggregation repo. Contains:
- V1 and V2 PDF manuals
- JMC PC software (v1.7.6, v2.4.4, v2.4.6)
- Step-by-step connection guide
- Notes on ESP32 Modbus integration attempts

The author was actively trying to use ESP32 via Modbus for non-pulse control but
was "overwhelmed by Modbus overhead" at time of last update (Feb 2024).

---

### JMC_servo_tuning (Python)

- **Repo**: https://github.com/bhowiebkr/JMC_servo_tuning
- **Local**: `docs/iHSV57-repos/JMC_servo_tuning/`

Python parameter extraction from JMC HTML manuals plus tuning guidance.

---

### ossm-rs (Rust - OSSM firmware)

- **Repo**: https://github.com/ossm-rs/ossm-rs
- **Local**: `docs/iHSV57-repos/ossm-rs/`

Complete Rust firmware for ESP32 that communicates with the 57AIM30 motor over
RS485 Modbus RTU. While this targets the 57AIM (not iHSV57), it demonstrates:
- 100 Hz position update rate over Modbus
- Proprietary function code 0x7B for absolute position
- Jerk-limited trajectory planning (Ruckig library)
- Full 32768-step encoder resolution
- Baud rate auto-upgrade from 19200 to 115200

---

### Other Tools

| Tool | Description |
|------|-------------|
| JMC-Servo-Parameter-Viewer | Qt app for viewing saved parameter XML files (bhowiebkr) |
| randomplum/jmc-servo | Qt app for JMC MCAC706 servo controller board |
| Serial Port Monitor | Commercial tool for sniffing serial traffic (https://www.serial-port-monitor.org/) - free trial |
| Modbus simulators | https://www.dalescott.net/modbus-development/ |

---

## Useful Links

- JMC Official Downloads: https://www.jmc-motor.com/download/list/17.html
- Rocketronics (German, guides + firmware): https://www.rocketronics.de
- Mark Rehorst's teardown blog: https://drmrehorst.blogspot.com/2020/04/ihsv-servomotor-information.html
- Blanch.org critical review: https://blanch.org/jmc-servo-review-and-guide/
- PrintNC Wiki IHSV57 page: https://wiki.printnc.info/en/electronics/Servos
- YouTube setup video: https://youtu.be/_9Q-VFesnA0
- PL2303 driver fixes: https://www.ifamilysoftware.com/Prolific_PL-2303_Code_10_Fix.html
