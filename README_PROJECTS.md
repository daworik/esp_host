# ESP32 Drone Hive - Project Structure

This repository contains **three separate ESP-IDF projects** for your drone hive system.

## 📁 Directory Structure

```
/workspace/
├── host_project/          # 🎮 HOST/CONTROLLER (Flash to master ESP32)
│   ├── CMakeLists.txt
│   └── main/
│       └── main.c         (master_drone_controller.c)
│
├── agent_project/         # 🚁 AGENT/DRONE (Flash to drone ESP32s)
│   ├── CMakeLists.txt
│   └── main/
│       └── main.c         (slave_motor_improved.c)
│
└── legacy_project/        # 📜 REFERENCE (original simple version)
    ├── CMakeLists.txt
    └── main/
        └── main.c         (wifi_eap_fast_main.c)
```

## 🔧 Building Instructions

### For Host/Controller ESP32:
```bash
cd /workspace/host_project
idf.py set-target esp32
idf.py menuconfig  # Configure WiFi, serial port, etc.
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### For Agent/Drone ESP32:
```bash
cd /workspace/agent_project
idf.py set-target esp32
idf.py menuconfig  # Configure WiFi, motor pins, MAC addresses
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

### For Legacy Reference:
```bash
cd /workspace/legacy_project
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## ⚙️ Configuration

### Host Project (`host_project/main/main.c`)
- Edit `drone_registry_t drones[]` array to add your drone MAC addresses
- Adjust `MAX_DRONES` constant if needed
- Configure UART port and baud rate in `app_main()`

### Agent Project (`agent_project/main/main.c`)
- Update `STEP_PIN`, `DIR_PIN`, `MS1_PIN`, `MS2_PIN` for your motor driver
- Set `DEFAULT_SPEED`, `MAX_PPS`, `ACCELERATION` values
- No MAC configuration needed (agents receive from any host)

## 📡 Communication Protocol

Both projects use ESP-NOW with JSON commands:

**Commands:**
- `forward` - Move forward at default speed
- `backward` - Move backward at default speed  
- `stop` - Stop immediately
- `set_speed <value>` - Set specific speed (-2000 to 2000)
- `query_status` - Request current status (speed, steps, link OK)
- `reset_steps` - Reset step counter to 0

**Protocol Features:**
- Request IDs for tracking responses
- Individual drone addressing by MAC
- Broadcast support (`all` command)
- Link timeout safety (auto-stop after 2s signal loss)
- Acknowledgment system with telemetry

## 🔍 Monitoring

Host outputs real-time telemetry:
```
[DRONE] Status update:
  MAC: aa:bb:cc:dd:ee:01
  Speed: 1500 PPS
  Steps: 12450
  Link: OK (RSSI: -45 dBm)
  Last seen: 0ms ago
```

## ⚠️ Important Notes

1. **One project per ESP32**: Flash `host_project` to ONE ESP32 (controller), `agent_project` to multiple ESP32s (drones)

2. **WiFi Channel**: Both host and agents must use the same WiFi channel (default: 1). Configure in `menuconfig` → Component config → Wi-Fi → Wi-Fi Channel

3. **Power**: Ensure adequate power supply for motor drivers and ESP32s (separate 5V/12V for motors, 3.3V regulated for ESP32)

4. **GPIO Pins**: Verify motor driver pin connections match the code or update in `agent_project/main/main.c`

## 🚀 Quick Start

1. Flash host code to master ESP32
2. Flash agent code to drone ESP32s
3. Power on all devices
4. Open host serial monitor
5. Send commands via UART: `drone_1 forward`, `all stop`, etc.

See `QUICKSTART.md` and `ADDITIONAL_IMPROVEMENTS.md` in the root for detailed guides.
