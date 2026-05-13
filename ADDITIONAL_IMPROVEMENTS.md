# Additional Improvements for ESP-NOW Drone Hive

## Current State Assessment
Your implementation is already production-ready with excellent fundamentals. Here are optional enhancements for specific use cases.

---

## 1. RSSI-Based Signal Quality Monitoring

**Problem:** No visibility into link quality before failures occur.

**Solution:** Add RSSI tracking to detect degrading connections early.

### Slave Side (add to `slave_motor_improved.c`):
```c
// In esp_now_recv_cb, after receiving command
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, 
                            const uint8_t *data, int data_len) {
    // ... existing code ...
    
    // Add RSSI to ACK
    int8_t rssi = recv_info->rssi;  // Available in recv_info
    cJSON_AddNumberToObject(ack, "rssi", rssi);
}
```

### Master Side (add to `master_drone_controller.c`):
```c
typedef struct {
    // ... existing fields ...
    int8_t last_rssi;
    uint8_t weak_signal_count;
} drone_t;

// In status report
ESP_LOGI(TAG, "%s: link=%s rssi=%d speed=%d steps=%u",
         drones[i].name,
         drones[i].link_ok ? "OK" : "LOST",
         drones[i].last_rssi,
         drones[i].current_speed,
         drones[i].steps);

// Warn on weak signal
if (drones[i].last_rssi < -80) {
    drones[i].weak_signal_count++;
    if (drones[i].weak_signal_count > 5) {
        ESP_LOGW(TAG, "Drone %s has weak signal (%d dBm)", 
                 drones[i].name, drones[i].last_rssi);
    }
} else {
    drones[i].weak_signal_count = 0;
}
```

---

## 2. Command Queue with Priority Levels

**Problem:** All commands treated equally; emergency stops may be delayed.

**Solution:** Implement priority queue for critical commands.

### Enhanced Command Structure:
```json
{
  "type": "cmd",
  "req_id": 123,
  "priority": "high",  // low, normal, high, critical
  "cmd": "stop",
  "timestamp": 1234567890
}
```

### Implementation:
```c
typedef enum {
    PRIORITY_LOW = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_HIGH = 2,
    PRIORITY_CRITICAL = 3
} command_priority_t;

typedef struct {
    uint8_t mac[6];
    char cmd[32];
    int32_t req_id;
    int32_t speed_param;
    command_priority_t priority;
    uint32_t timestamp;
} queued_command_t;

static QueueHandle_t command_queue;

// In master, create queue
command_queue = xQueueCreate(20, sizeof(queued_command_t));

// Critical commands bypass queue
if (priority == PRIORITY_CRITICAL) {
    send_command_to_drone_immediate(mac, cmd, req_id, speed);
} else {
    xQueueSend(command_queue, &cmd_struct, portMAX_DELAY);
}
```

---

## 3. Battery/Telemetry Data Support

**Problem:** No battery monitoring or sensor data from drones.

**Solution:** Extend ACK packet with telemetry fields.

### Slave Side Addition:
```c
// Add ADC reading for battery
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0
#define BATTERY_VOLTAGE_SCALE 0.000732f  // Depends on voltage divider

static float read_battery_voltage(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11);
    uint32_t raw = adc1_get_raw(BATTERY_ADC_CHANNEL);
    return raw * BATTERY_VOLTAGE_SCALE * 2.0f;  // 2:1 divider example
}

// In send_ack:
float battery_v = read_battery_voltage();
cJSON_AddNumberToObject(ack, "battery", battery_v);

// Optional: Add temperature
temperature_sensor_get_temperature(&temp);
cJSON_AddNumberToObject(ack, "temp", temp);
```

### Master Display:
```
I (7890) MASTER: === DRONE STATUS ===
I (7891) MASTER: drone_1: link=OK rssi=-65 bat=11.2V temp=42C spd=1500 steps=3421
I (7892) MASTER: drone_2: link=OK rssi=-78 bat=10.8V temp=45C spd=0 steps=1205
```

---

## 4. Formation Control Commands

**Problem:** Coordinating multiple drones requires sending individual commands.

**Solution:** Add formation primitives for synchronized movement.

### New Commands:
```json
// Formation: line
{
  "type": "formation",
  "pattern": "line",
  "spacing": 50,
  "direction": "forward",
  "speed": 1000
}

// Formation: circle
{
  "type": "formation",
  "pattern": "circle",
  "radius": 100,
  "clockwise": true,
  "speed": 800
}
```

### Master Implementation:
```c
typedef struct {
    uint8_t position_in_formation;
    float offset_x, offset_y;
    bool formation_active;
} formation_state_t;

static formation_state_t formations[MAX_DRONES];

void execute_formation(const char* pattern, float param1, float param2) {
    xSemaphoreTake(drone_mutex, portMAX_DELAY);
    for (int i = 0; i < drone_count; i++) {
        // Calculate individual offsets based on position
        float speed_mod = 1.0f;
        int32_t target_speed = DEFAULT_SPEED;
        
        if (strcmp(pattern, "circle") == 0) {
            // Outer drones need higher speed
            speed_mod = 1.0f + (formations[i].position_in_formation * 0.1f);
            target_speed = (int32_t)(DEFAULT_SPEED * speed_mod);
        }
        
        send_command_to_drone(drones[i].mac, "set_speed", 
                             ++global_req_id, target_speed);
    }
    xSemaphoreGive(drone_mutex);
}
```

---

## 5. OTA Firmware Update Support

**Problem:** Need physical access to update slave firmware.

**Solution:** Implement firmware update over ESP-NOW.

### Protocol Extension:
```json
// Start update
{
  "type": "ota",
  "action": "start",
  "firmware_version": "1.2.3",
  "total_size": 524288,
  "chunk_size": 1024
}

// Send chunk
{
  "type": "ota_chunk",
  "chunk_id": 42,
  "data": "<base64 encoded binary>"
}

// Complete
{
  "type": "ota",
  "action": "complete",
  "checksum": "sha256_hash"
}
```

### Slave Side Handler:
```c
static esp_ota_handle_t ota_handle;
static const esp_partition_t *update_partition;
static bool ota_in_progress = false;
static uint32_t ota_received_bytes = 0;

static void handle_ota_command(cJSON *json) {
    cJSON *action = cJSON_GetObjectItem(json, "action");
    
    if (strcmp(action->valuestring, "start") == 0) {
        update_partition = esp_ota_get_next_update_partition(NULL);
        esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
        ota_in_progress = true;
        ota_received_bytes = 0;
        ESP_LOGI(TAG, "OTA started");
    }
    // Handle chunks and completion...
}
```

---

## 6. Time Synchronization for Coordinated Actions

**Problem:** Drones act on commands at different times due to latency.

**Solution:** Implement time sync for scheduled maneuvers.

### Master Sends Sync:
```c
#include "esp_sntp.h"

static uint64_t get_network_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

// Broadcast time sync every 10 seconds
{
  "type": "sync",
  "timestamp_us": 1234567890123456,
  "drift_correction": 150
}
```

### Scheduled Command Execution:
```json
{
  "type": "cmd",
  "cmd": "forward",
  "execute_at_us": 1234567900000000,  // Execute at specific time
  "req_id": 123
}
```

### Slave Side:
```c
static uint64_t slave_time_offset = 0;

if (cJSON_HasObjectItem(json, "execute_at_us")) {
    uint64_t execute_time = cJSON_GetObjectItem(json, "execute_at_us")->valuedouble;
    uint64_t now = esp_timer_get_time() + slave_time_offset;
    
    if (execute_time > now) {
        // Schedule for later
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS((execute_time - now) / 1000));
    }
}
// Execute command
```

---

## 7. Mesh Relay for Extended Range

**Problem:** Direct ESP-NOW range limited (~200m line of sight).

**Solution:** Allow drones to relay messages for out-of-range peers.

### Routing Table:
```c
typedef struct {
    uint8_t destination_mac[6];
    uint8_t next_hop_mac[6];
    uint8_t hop_count;
    uint32_t last_seen;
} routing_entry_t;

static routing_entry_t routing_table[10];
```

### Relay Logic:
```c
static void esp_now_recv_cb(...) {
    cJSON *relay = cJSON_GetObjectItem(json, "relay");
    
    if (cJSON_IsTrue(relay)) {
        // This is a relayed packet
        // Check if we're the final destination
        if (memcmp(dest_mac, our_mac, 6) != 0) {
            // Forward to next hop
            find_next_hop_and_forward(data, data_len);
            return;
        }
    }
    
    // Normal processing...
}
```

---

## 8. Configuration via NVS (Persistent Settings)

**Problem:** Hardcoded values require recompilation.

**Solution:** Store configuration in NVS flash.

```c
#include "nvs.h"

typedef struct {
    int32_t max_speed;
    int32_t acceleration;
    uint8_t drone_id;
    char drone_name[16];
    bool invert_direction;
} drone_config_t;

static void load_config(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("drone_cfg", NVS_READONLY, &h);
    
    if (err == ESP_OK) {
        nvs_get_i32(h, "max_speed", &motor.max_speed);
        nvs_get_i32(h, "acceleration", &motor.acceleration);
        char name[16];
        nvs_get_str(h, "name", name, sizeof(name));
        strncpy(motor.name, name, 15);
        nvs_close(h);
    } else {
        // Use defaults
    }
}

static void save_config(void) {
    nvs_handle_t h;
    nvs_open("drone_cfg", NVS_READWRITE, &h);
    nvs_set_i32(h, "max_speed", motor.target_speed);
    nvs_commit(h);
    nvs_close(h);
}
```

---

## 9. Web Interface for Master Control

**Bonus:** Add WiFi AP mode on master for browser-based control.

```c
#include "esp_http_server.h"

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_ctrl = {
            .uri = "/api/control",
            .method = HTTP_POST,
            .handler = control_handler,
        };
        httpd_register_uri_handler(server, &uri_ctrl);
    }
    return server;
}

// Access via browser: http://192.168.4.1
// POST /api/control {"drone": "drone_1", "cmd": "forward"}
```

---

## Implementation Priority

| Feature | Complexity | Impact | Recommended For |
|---------|-----------|--------|-----------------|
| RSSI Monitoring | Low | Medium | All deployments |
| Battery Telemetry | Low | High | Long-duration flights |
| Command Priorities | Medium | High | Safety-critical ops |
| Formation Control | High | Medium | Swarm demonstrations |
| Time Synchronization | Medium | High | Coordinated maneuvers |
| OTA Updates | High | High | Large fleets |
| Mesh Relay | Very High | Medium | Extended range needs |
| Web Interface | Medium | Low | Ground station alternative |

---

## Quick Win: Add RSSI + Battery Now

Minimal changes for immediate benefit:

### 1. Slave (`slave_motor_improved.c` line ~175):
```c
// Add after cJSON_AddNumberToObject(ack, "speed", motor.current_speed);
cJSON_AddNumberToObject(ack, "rssi", recv_info->rssi);  // Need to pass rssi to send_ack
```

### 2. Master (`master_drone_controller.c` line ~36):
```c
// Add to drone_t struct
int8_t last_rssi;

// Line ~168, add after steps:
cJSON *rssi = cJSON_GetObjectItem(json, "rssi");
if (cJSON_IsNumber(rssi)) {
    drones[drone_idx].last_rssi = (int8_t)rssi->valuedouble;
}

// Line ~299, update status output:
ESP_LOGI(TAG, "%s: link=%s rssi=%dBm speed=%d steps=%u",
         drones[i].name,
         drones[i].link_ok ? "OK" : "LOST",
         drones[i].last_rssi,
         drones[i].current_speed,
         drones[i].steps);
```

This gives you immediate visibility into link quality!

---

## Testing Recommendations

1. **Range Testing**: Gradually increase distance, monitor RSSI threshold
2. **Interference Testing**: Operate near WiFi routers, microwave ovens
3. **Multi-Drone Stress Test**: 10+ drones simultaneous commands
4. **Link Loss Simulation**: Power off drones mid-operation
5. **Command Flood Test**: Rapid-fire commands to test queue handling

---

## Conclusion

Your current implementation is solid for production use. The enhancements above are optional depending on your specific requirements:

- **Start with RSSI + Battery** (30 min implementation)
- **Add Command Priorities** if safety is critical
- **Consider OTA** for fleet management
- **Mesh Relay** only if range is a proven limitation

Let me know which features you'd like me to implement with full code examples!
