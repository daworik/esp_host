# Build & Flash Guide

## ✅ Project Structure Created

Your repository now has **3 separate ESP-IDF projects**:

```
/workspace/
├── host_project/      # Controller (1 ESP32)
├── agent_project/     # Drones (multiple ESP32s)
└── legacy_project/    # Reference code
```

## 🔧 Building on Your Development Machine

Since ESP-IDF is not installed in this environment, you'll build on your local machine:

### Option 1: Copy Projects to Local Machine

```bash
# On your local machine with ESP-IDF installed:
git clone <your-repo-url>
cd <repo>/host_project
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# Then for agents:
cd ../agent_project
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

### Option 2: Use ESP-IDF Docker

```bash
docker run --rm -v $PWD:/workspace -w /workspace/espressif/idf docker.io/espressif/idf:release-v5.2 \
  bash -c "cd host_project && idf.py set-target esp32 && idf.py build"
```

### Option 3: Use VS Code ESP-IDF Extension

1. Install VS Code + ESP-IDF extension
2. Open `host_project` folder in VS Code
3. Click "Build" button
4. Click "Flash and Monitor"
5. Repeat for `agent_project`

## 📋 Pre-Flash Configuration

### For Host (`host_project`):
1. Run `idf.py menuconfig`
2. Configure:
   - **Serial port**: Your USB-to-UART adapter
   - **WiFi channel**: Component config → Wi-Fi → Channel (default: 1)
   - **Drone MAC addresses**: Edit `main/main.c`, find `drone_registry_t drones[]`

### For Agent (`agent_project`):
1. Run `idf.py menuconfig`
2. Configure:
   - **Serial port**: Your USB-to-UART adapter
   - **WiFi channel**: Must match host!
   - **Motor pins**: Edit `main/main.c` if your wiring differs:
     ```c
     #define STEP_PIN    GPIO_NUM_5
     #define DIR_PIN     GPIO_NUM_6
     #define MS1_PIN     GPIO_NUM_12
     #define MS2_PIN     GPIO_NUM_10
     ```

## 🚀 Flash Sequence

1. **Flash Host First**:
   ```bash
   cd host_project
   idf.py -p /dev/ttyUSB0 flash monitor
   ```
   Wait for "Host ready" message

2. **Flash Agents** (one by one):
   ```bash
   cd agent_project
   idf.py -p /dev/ttyUSB1 flash monitor
   ```
   Repeat for each drone ESP32

3. **Test Communication**:
   In host serial monitor, type:
   ```
   all query_status
   ```
   You should see responses from all connected drones

## ⚠️ Common Issues

### "No devices found"
- Check USB cables
- Verify correct `/dev/ttyUSB*` port
- Try `ls /dev/ttyUSB*` to list available ports

### "WiFi init failed"
- Ensure WiFi channel matches between host and agents
- Check `menuconfig` → Wi-Fi settings

### "Motor not spinning"
- Verify GPIO pin connections
- Check motor driver power supply
- Confirm `DEFAULT_SPEED` is high enough (try 1000+)

### "ESP-NOW timeout"
- Reduce distance between ESP32s
- Check antenna connections
- Verify both devices are powered adequately

## 📊 Expected Output

**Host Console:**
```
I (1234) HOST: Host starting...
I (2345) HOST: ESP-NOW initialized
I (3456) HOST: Registered 3 drones
I (4567) HOST: Ready for commands
> all query_status
[DRONE] Status: MAC=aa:bb:cc:dd:ee:01 Speed=0 Steps=0 Link=OK
[DRONE] Status: MAC=aa:bb:cc:dd:ee:02 Speed=0 Steps=0 Link=OK
[DRONE] Status: MAC=aa:bb:cc:dd:ee:03 Speed=0 Steps=0 Link=OK
```

**Agent Console:**
```
I (1234) MOTOR_SLAVE: Slave starting
I (2345) MOTOR_SLAVE: GPIO initialized
I (3456) MOTOR_SLAVE: ESP-NOW slave ready
I (4567) MOTOR_SLAVE: Ready
I (5678) MOTOR_SLAVE: CMD: {"req_id":1,"cmd":"query_status"}
```

## 🛠️ Next Steps

After successful build and flash:
1. Test basic movement commands
2. Calibrate motor speeds
3. Add more drones to registry
4. Implement advanced features (see `ADDITIONAL_IMPROVEMENTS.md`)
