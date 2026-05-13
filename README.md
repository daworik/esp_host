# ESP32 Drone Hive - ESP-NOW Orchestration System

| Supported Targets | ESP32 | ESP32-C3 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- |

## 🎯 Overview

This project implements a **hive-like drone orchestration system** using ESP-NOW protocol. One ESP32 acts as a **Host/Controller** that sends commands to multiple **Agent/Drone** ESP32s, each controlling a motor.

## 📁 Project Structure

```
/workspace/
├── host_project/          # 🎮 HOST/CONTROLLER (Flash to master ESP32)
│   ├── CMakeLists.txt
│   └── main/main.c        (master drone controller)
│
├── agent_project/         # 🚁 AGENT/DRONE (Flash to drone ESP32s)
│   ├── CMakeLists.txt
│   └── main/main.c        (motor slave with improved features)
│
├── legacy_project/        # 📜 REFERENCE (original simple version)
│   ├── CMakeLists.txt
│   └── main/main.c
│
├── QUICKSTART.md          # Quick setup guide
├── BUILD_GUIDE.md         # Detailed build instructions
├── README_PROJECTS.md     # Project structure explanation
└── ADDITIONAL_IMPROVEMENTS.md  # Advanced feature proposals
```

## ✨ Key Features

### Host Controller
- ✅ Multi-drone registry with MAC address management
- ✅ Individual and broadcast command support
- ✅ Real-time telemetry monitoring (speed, steps, link status)
- ✅ Link timeout detection with auto-safety stop
- ✅ UART command interface for manual control
- ✅ JSON-based bidirectional protocol with request tracking

### Agent Drone
- ✅ Smooth acceleration/deceleration profiles
- ✅ Thread-safe motor control with mutex protection
- ✅ ESP-NOW command receiver with JSON parsing
- ✅ Status reporting (speed, step count, link health)
- ✅ Watchdog timer for link-loss safety
- ✅ Configurable GPIO pins for motor drivers

## 🚀 Quick Start

### 1. Build Host Project
```bash
cd host_project
idf.py set-target esp32
idf.py menuconfig  # Configure WiFi channel, add drone MACs
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Build Agent Project
```bash
cd agent_project
idf.py set-target esp32
idf.py menuconfig  # Configure WiFi channel (must match host!)
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

### 3. Test Commands
In host serial monitor:
```
all forward          # All drones move forward
drone_1 backward     # Specific drone moves backward
all stop             # Emergency stop
drone_1 query_status # Get telemetry from drone_1
```

## 📡 Communication Protocol

**Commands:**
- `forward` / `backward` / `stop` - Basic movement
- `set_speed <value>` - Set speed (-2000 to 2000 PPS)
- `query_status` - Request telemetry
- `reset_steps` - Reset step counter

**Example JSON:**
```json
{"req_id": 1, "cmd": "forward"}
{"req_id": 1, "status": "OK", "speed": 1500, "steps": 1234}
```

## ⚙️ Configuration

### Host (`host_project/main/main.c`)
Edit drone MAC addresses:
```c
static drone_registry_t drones[MAX_DRONES] = {
    {.id = "drone_1", .mac = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01}},
    {.id = "drone_2", .mac = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02}},
    // Add more drones...
};
```

### Agent (`agent_project/main/main.c`)
Configure motor pins:
```c
#define STEP_PIN    GPIO_NUM_5
#define DIR_PIN     GPIO_NUM_6
#define MS1_PIN     GPIO_NUM_12
#define MS2_PIN     GPIO_NUM_10
```

## 📖 Documentation

- **[QUICKSTART.md](QUICKSTART.md)** - Hardware setup, flashing, basic usage
- **[BUILD_GUIDE.md](BUILD_GUIDE.md)** - Detailed build instructions, troubleshooting
- **[README_PROJECTS.md](README_PROJECTS.md)** - Project organization explanation
- **[ADDITIONAL_IMPROVEMENTS.md](ADDITIONAL_IMPROVEMENTS.md)** - Advanced features (RSSI, battery, formations, OTA)

## 🔧 Requirements

- ESP-IDF v5.0 or later
- ESP32 dev boards (1 for host, multiple for agents)
- Stepper motors with drivers (A4988, DRV8825, etc.)
- USB-to-UART adapters for programming

## ⚠️ Important Notes

1. **WiFi Channel**: Host and all agents must use the same WiFi channel
2. **Power**: Use separate power supplies for motors and ESP32s
3. **Range**: ESP-NOW works up to ~100m line-of-sight
4. **MAC Addresses**: Update host code with actual agent MAC addresses

## 🛠️ Legacy Code

The original `wifi_eap_fast_main.c` is preserved in `legacy_project/` for reference. It demonstrates basic ESP-NOW without the advanced features.

---

**Original Example**: WPA2 Enterprise (see legacy_project for reference)
