# ESP-NOW Drone Hive - Quick Start Guide

## Architecture Overview

```
┌─────────────────┐     ESP-NOW      ┌─────────────────┐
│   MASTER (Host) │◄────────────────►│  SLAVE (Drone)  │
│   ESP32 + USB   │   Bidirectional  │  ESP32 + Motor  │
│   Serial Port   │   Unicast ACKs   │  Controller     │
└─────────────────┘                  └─────────────────┘
       ▲                                    ▲
       │                                    │
       ▼                                    ▼
  PC/Terminal                          Stepper Motor
  Commands                             (via driver)
```

## Files in This Repo

| File | Purpose | Use For |
|------|---------|---------|
| `main/master_drone_controller.c` | **Host/Controller** code | Flash to master ESP32 |
| `main/slave_motor_improved.c` | **Agent/Drone** code | Flash to each slave ESP32 |
| `main/wifi_eap_fast_main.c` | Original simple master | Reference only |

---

## Step-by-Step Setup

### 1. Flash Slave Code to Drone ESP32

```bash
cd /workspace
cp main/slave_motor_improved.c main/main.c
idf.py -p /dev/ttyUSB0 flash monitor
```

**Note the MAC address** from boot log:
```
I (1234) MOTOR_SLAVE: Ready - MAC: aa:bb:cc:dd:ee:01
```

Repeat for each drone, noting each MAC address.

### 2. Configure Master with Drone MACs

Edit `main/master_drone_controller.c`, lines 47-51:

```c
static const uint8_t DRONE_MACS[][6] = {
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},  // Replace with actual MACs
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},
};
```

### 3. (Optional) Set Encryption Key

Both master and slave must use the same PMK if encryption enabled.

**In `slave_motor_improved.c` line 293:**
```c
ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"YOUR_16_CHAR_KEY"));  // Exactly 16 chars!
```

**In `master_drone_controller.c` line 187:** Uncomment and use same key:
```c
ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"YOUR_16_CHAR_KEY"));
```

### 4. Flash Master Code

```bash
cp main/master_drone_controller.c main/main.c
idf.py -p /dev/ttyUSB1 flash monitor
```

---

## Usage Examples

### Connect to Master via Serial
```bash
screen /dev/ttyUSB1 115200
# or
minicom -D /dev/ttyUSB1 -b 115200
```

### Send Commands

**Individual drone control:**
```
drone_1 forward
drone_1 stop
drone_2 backward
drone_2 set_speed 1200
drone_3 query_status
drone_1 reset_steps
```

**Broadcast to all:**
```
all stop
all forward
broadcast query_status
```

### Expected Output

```
I (1234) MASTER: Registered drone drone_1: aa:bb:cc:dd:ee:01
I (2345) MASTER: UART RX: drone_1 forward
I (2350) MASTER: Drone drone_1 ack: ok
I (7890) MASTER: === DRONE STATUS ===
I (7891) MASTER: drone_1: link=OK speed=1500 steps=3421
I (7892) MASTER: drone_2: link=LOST speed=0 steps=0
```

---

## Available Commands

| Command | Parameters | Description |
|---------|-----------|-------------|
| `forward` | - | Move at default speed (1500 PPS) |
| `backward` | - | Reverse at default speed |
| `stop` | - | Emergency stop |
| `set_speed` | speed (-2000 to 2000) | Variable speed control |
| `query_status` | - | Request current state (returns speed + steps) |
| `reset_steps` | - | Reset step counter to zero |

---

## Safety Features

### Link Timeout Protection
- Slaves automatically stop if no command received within **2 seconds**
- Master monitors link status and reports "LOST" when connection breaks
- Prevents runaway drones on signal loss

### Thread-Safe Operation
- Mutex protection prevents race conditions
- Safe concurrent access from motor task and ESP-NOW callback

### Acceleration Ramping
- Smooth acceleration/deceleration (500 PPS/s)
- Prevents missed steps and mechanical stress

---

## Troubleshooting

### No ACK Received
1. Check MAC addresses match exactly
2. Verify both devices powered and within range (~100m line of sight)
3. Check serial logs on slave for "CMD:" messages
4. Try moving devices closer

### Link Lost Errors
1. Increase `LINK_TIMEOUT_MS` in slave code (line 34)
2. Check for WiFi interference
3. Verify antennas connected properly

### Compilation Errors
```bash
# Clean build
idf.py fullclean
idf.py build

# Check IDF version
idf.py --version  # Should be 4.4+ or 5.x
```

### Motor Not Moving
1. Verify GPIO pins match your stepper driver (lines 27-30 in slave)
2. Check power supply to motor driver
3. Test with manual GPIO toggle first
4. Verify `DEFAULT_SPEED` is appropriate for your motor

---

## Hardware Connections

### Slave ESP32 to Stepper Driver (e.g., A4988)

| ESP32 Pin | Driver Pin |
|-----------|------------|
| GPIO 5 | STEP |
| GPIO 6 | DIR |
| GPIO 12 | MS1 (microstep select) |
| GPIO 10 | MS2 (microstep select) |
| 3.3V | ENABLE (active low, pull up to disable) |
| GND | GND |
| 5V (or external) | VDD |
| Motor Power | VMOT |

**Note:** Adjust pin definitions in `slave_motor_improved.c` if using different GPIOs.

---

## Performance Specifications

| Parameter | Value | Configurable |
|-----------|-------|--------------|
| Max Speed | 2000 PPS | `MAX_PPS` (line 31) |
| Default Speed | 1500 PPS | `DEFAULT_SPEED` (line 33) |
| Acceleration | 500 PPS/s | `ACCELERATION` (line 32) |
| Link Timeout | 2000 ms | `LINK_TIMEOUT_MS` (line 34) |
| Control Loop | 50 Hz | Fixed |
| Max Drones | 10 | `MAX_DRONES` in master |

---

## Next Steps

1. **Test basic operation** with single drone
2. **Add more drones** by flashing slave code and registering MACs
3. **Implement telemetry** (see ADDITIONAL_IMPROVEMENTS.md)
4. **Build ground station** with Python/Node.js serial interface
5. **Add sensors** (battery, IMU) to slaves

---

## Further Reading

- `IMPROVEMENTS.md` - Detailed explanation of protocol improvements
- `ADDITIONAL_IMPROVEMENTS.md` - Advanced features (RSSI, OTA, formations)
- [ESP-NOW Official Docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)

---

## Support

For issues or questions:
1. Check serial logs on both master and slave
2. Verify MAC address configuration
3. Test with minimal setup (1 master + 1 slave)
4. Review ESP-NOW peer registration in logs

Happy flying! 🚁
