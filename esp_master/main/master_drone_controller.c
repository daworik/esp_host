// master_drone_controller.c
// ESP-NOW Drone Hive Controller with encryption, acknowledgments, and individual addressing
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#define TAG "MASTER"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 256

// ESP-NOW Encryption Key (32 bytes for PMK)
static const uint8_t esp_now_key[ESP_NOW_KEY_LEN] = {
    0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x7a, 0x8b,
    0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03,
    0x14, 0x25, 0x36, 0x47, 0x58, 0x69, 0x7a, 0x8b,
    0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03
};

// Maximum number of drones
#define MAX_DRONES 10
#define DRONE_HEARTBEAT_TIMEOUT_MS 3000
#define COMMAND_RETRY_COUNT 3
#define COMMAND_TIMEOUT_MS 500

// Message types
typedef enum {
    MSG_TYPE_COMMAND = 0,
    MSG_TYPE_ACK = 1,
    MSG_TYPE_TELEMETRY = 2,
    MSG_TYPE_DISCOVERY = 3,
    MSG_TYPE_DISCOVERY_RESPONSE = 4
} msg_type_t;

// Command types
typedef enum {
    CMD_TAKEOFF = 0,
    CMD_LAND = 1,
    CMD_FORWARD = 2,
    CMD_BACKWARD = 3,
    CMD_LEFT = 4,
    CMD_RIGHT = 5,
    CMD_UP = 6,
    CMD_DOWN = 7,
    CMD_STOP = 8,
    CMD_SET_SPEED = 9,
    CMD_SET_ID = 10,
    CMD_DISCOVER = 11
} command_t;

// Packed message structure (no padding)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;      // Message type
    uint8_t drone_id;      // Target drone ID (0 = broadcast)
    uint8_t command;       // Command code
    uint8_t seq_num;       // Sequence number for acknowledgment
    int16_t param1;        // Parameter 1 (e.g., speed)
    int16_t param2;        // Parameter 2 (e.g., duration)
    uint32_t timestamp;    // Timestamp
} drone_message_t;

// Telemetry structure
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t drone_id;
    uint16_t battery_mv;     // Battery voltage in mV
    int16_t speed;           // Current speed
    uint16_t uptime_sec;     // Uptime in seconds
    int8_t rssi;             // Signal strength
    uint8_t status_flags;    // Status flags
    uint32_t timestamp;
} telemetry_message_t;

// Drone state
typedef struct {
    uint8_t id;
    uint8_t mac_addr[6];
    bool active;
    uint32_t last_heartbeat;
    uint8_t last_seq_num;
    int8_t rssi;
} drone_t;

static drone_t drones[MAX_DRONES];
static uint8_t drone_count = 0;
static uint8_t current_seq = 0;
static QueueHandle_t ack_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;

typedef struct {
    uint8_t drone_id;
    uint8_t seq_num;
    bool success;
} ack_result_t;

// Forward declarations
static esp_err_t send_command(uint8_t drone_id, uint8_t command, int16_t param1, int16_t param2);
static void process_ack(const uint8_t *mac, const drone_message_t *msg);
static void process_telemetry(const uint8_t *mac, const telemetry_message_t *msg);

// Initialize UART
static void uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_cfg);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// Find drone by MAC address
static int find_drone_by_mac(const uint8_t *mac) {
    for (int i = 0; i < drone_count; i++) {
        if (memcmp(drones[i].mac_addr, mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

// Find drone by ID
static int find_drone_by_id(uint8_t id) {
    for (int i = 0; i < drone_count; i++) {
        if (drones[i].id == id && drones[i].active) {
            return i;
        }
    }
    return -1;
}

// Add new drone
static void add_drone(const uint8_t *mac, uint8_t id) {
    if (drone_count >= MAX_DRONES) {
        ESP_LOGW(TAG, "Max drones reached");
        return;
    }
    
    // Check if already exists
    if (find_drone_by_mac(mac) >= 0) {
        return;
    }
    
    memcpy(drones[drone_count].mac_addr, mac, 6);
    drones[drone_count].id = id;
    drones[drone_count].active = true;
    drones[drone_count].last_heartbeat = esp_timer_get_time() / 1000;
    drones[drone_count].last_seq_num = 0;
    drones[drone_count].rssi = -100;
    
    drone_count++;
    ESP_LOGI(TAG, "Drone added: ID=%u, MAC=%02x:%02x:%02x:%02x:%02x:%02x", 
             id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Send data with encryption
static esp_err_t send_encrypted(const uint8_t *mac, const void *data, size_t len) {
    esp_now_peer_info_t peer = {
        .peer_addr = {0},
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = true,
    };
    memcpy(peer.peer_addr, mac, 6);
    memcpy(peer.lmk, esp_now_key, ESP_NOW_KEY_LEN);
    
    // Check if peer exists, add if not
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_add_peer(&peer);
    }
    
    return esp_now_send(mac, data, len);
}

// Send command to drone(s)
static esp_err_t send_command(uint8_t drone_id, uint8_t command, int16_t param1, int16_t param2) {
    drone_message_t msg;
    msg.msg_type = MSG_TYPE_COMMAND;
    msg.drone_id = drone_id;
    msg.command = command;
    msg.seq_num = current_seq++;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.timestamp = esp_timer_get_time() / 1000;
    
    uint8_t dest_mac[6];
    
    if (drone_id == 0) {
        // Broadcast
        memset(dest_mac, 0xFF, 6);
        ESP_LOGI(TAG, "Broadcast command: %u, params=(%d, %d)", command, param1, param2);
    } else {
        // Individual drone
        int idx = find_drone_by_id(drone_id);
        if (idx < 0) {
            ESP_LOGE(TAG, "Drone ID %u not found", drone_id);
            return ESP_ERR_NOT_FOUND;
        }
        memcpy(dest_mac, drones[idx].mac_addr, 6);
        ESP_LOGI(TAG, "Command to drone %u: %u, params=(%d, %d)", 
                 drone_id, command, param1, param2);
    }
    
    // Send with retries
    for (int retry = 0; retry < COMMAND_RETRY_COUNT; retry++) {
        esp_err_t err = send_encrypted(dest_mac, &msg, sizeof(msg));
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGE(TAG, "Failed to send command after %d retries", COMMAND_RETRY_COUNT);
    return ESP_FAIL;
}

// ESP-NOW receive callback
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    const uint8_t *mac = recv_info->src_addr;
    
    if (data_len < sizeof(drone_message_t)) {
        return;
    }
    
    // Check message type
    uint8_t msg_type = data[0];
    
    if (msg_type == MSG_TYPE_ACK && data_len >= sizeof(drone_message_t)) {
        drone_message_t msg;
        memcpy(&msg, data, sizeof(msg));
        process_ack(mac, &msg);
    } else if (msg_type == MSG_TYPE_TELEMETRY && data_len >= sizeof(telemetry_message_t)) {
        telemetry_message_t msg;
        memcpy(&msg, data, sizeof(msg));
        process_telemetry(mac, &msg);
    } else if (msg_type == MSG_TYPE_DISCOVERY_RESPONSE) {
        drone_message_t msg;
        memcpy(&msg, data, sizeof(msg));
        add_drone(mac, msg.drone_id);
    }
}

// Process acknowledgment
static void process_ack(const uint8_t *mac, const drone_message_t *msg) {
    int idx = find_drone_by_mac(mac);
    if (idx >= 0) {
        drones[idx].last_heartbeat = esp_timer_get_time() / 1000;
        drones[idx].last_seq_num = msg->seq_num;
        
        ack_result_t result = {
            .drone_id = msg->drone_id,
            .seq_num = msg->seq_num,
            .success = true
        };
        xQueueSend(ack_queue, &result, 0);
        
        ESP_LOGD(TAG, "ACK from drone %u, seq=%u", msg->drone_id, msg->seq_num);
    }
}

// Process telemetry
static void process_telemetry(const uint8_t *mac, const telemetry_message_t *msg) {
    int idx = find_drone_by_mac(mac);
    if (idx >= 0) {
        drones[idx].last_heartbeat = esp_timer_get_time() / 1000;
        drones[idx].rssi = msg->rssi;
        
        xQueueSend(telemetry_queue, msg, 0);
        
        ESP_LOGI(TAG, "Telemetry from drone %u: battery=%umV, speed=%d, uptime=%us, RSSI=%d",
                 msg->drone_id, msg->battery_mv, msg->speed, msg->uptime_sec, msg->rssi);
    }
}

// Initialize ESP-NOW Master
static void espnow_master_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk(esp_now_key));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    ESP_LOGI(TAG, "ESP-NOW master initialized with encryption");
}

// Command parser task
static void command_parser_task(void *arg) {
    uint8_t buffer[BUF_SIZE];
    
    while (1) {
        int len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buffer[len] = '\0';
            
            // Remove newline
            while (len > 0 && (buffer[len-1] == '\n' || buffer[len-1] == '\r')) {
                buffer[--len] = '\0';
            }
            
            ESP_LOGI(TAG, "Received: %s", buffer);
            
            // Parse commands: "CMD [ID] [PARAM1] [PARAM2]"
            char cmd[32];
            uint8_t drone_id = 0;  // 0 = broadcast
            int16_t param1 = 0, param2 = 0;
            
            int parsed = sscanf((char*)buffer, "%31s %hhu %hd %hd", cmd, &drone_id, &param1, &param2);
            
            if (parsed < 1) continue;
            
            uint8_t command = CMD_STOP;
            
            if (strcmp(cmd, "takeoff") == 0) command = CMD_TAKEOFF;
            else if (strcmp(cmd, "land") == 0) command = CMD_LAND;
            else if (strcmp(cmd, "forward") == 0) command = CMD_FORWARD;
            else if (strcmp(cmd, "backward") == 0) command = CMD_BACKWARD;
            else if (strcmp(cmd, "left") == 0) command = CMD_LEFT;
            else if (strcmp(cmd, "right") == 0) command = CMD_RIGHT;
            else if (strcmp(cmd, "up") == 0) command = CMD_UP;
            else if (strcmp(cmd, "down") == 0) command = CMD_DOWN;
            else if (strcmp(cmd, "stop") == 0) command = CMD_STOP;
            else if (strcmp(cmd, "speed") == 0) command = CMD_SET_SPEED;
            else if (strcmp(cmd, "setid") == 0) command = CMD_SET_ID;
            else if (strcmp(cmd, "discover") == 0) {
                send_command(0, CMD_DISCOVER, 0, 0);
                continue;
            } else {
                ESP_LOGW(TAG, "Unknown command: %s", cmd);
                continue;
            }
            
            send_command(drone_id, command, param1, param2);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Acknowledgment monitor task
static void ack_monitor_task(void *arg) {
    ack_result_t result;
    
    while (1) {
        if (xQueueReceive(ack_queue, &result, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (result.success) {
                ESP_LOGD(TAG, "Command acknowledged by drone %u", result.drone_id);
            } else {
                ESP_LOGW(TAG, "Command failed for drone %u", result.drone_id);
            }
        }
        
        // Check for heartbeat timeouts
        uint32_t now = esp_timer_get_time() / 1000;
        for (int i = 0; i < drone_count; i++) {
            if (drones[i].active && 
                (now - drones[i].last_heartbeat) > DRONE_HEARTBEAT_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Drone %u heartbeat timeout!", drones[i].id);
                // Could mark as inactive or trigger reconnection
            }
        }
    }
}

// Telemetry display task
static void telemetry_task(void *arg) {
    telemetry_message_t msg;
    
    while (1) {
        if (xQueueReceive(telemetry_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Display telemetry
            printf("\r[D%u] Batt: %4dmV | Speed: %4d | Up: %5us | RSSI: %3d dBm   ",
                   msg.drone_id, msg.battery_mv, msg.speed, msg.uptime_sec, msg.rssi);
            fflush(stdout);
        }
        
        // Periodic status summary
        static uint32_t last_summary = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - last_summary > 5000) {
            ESP_LOGI(TAG, "Active drones: %u/%u", drone_count, MAX_DRONES);
            last_summary = now;
        }
    }
}

// Discovery task - periodically send discovery requests
static void discovery_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Every 10 seconds
        
        if (drone_count == 0) {
            ESP_LOGI(TAG, "Sending discovery request...");
            send_command(0, CMD_DISCOVER, 0, 0);
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Initialize queues
    ack_queue = xQueueCreate(10, sizeof(ack_result_t));
    telemetry_queue = xQueueCreate(10, sizeof(telemetry_message_t));
    
    // Initialize hardware
    uart_init();
    espnow_master_init();
    
    // Create tasks
    xTaskCreate(command_parser_task, "cmd_parser", 4096, NULL, 5, NULL);
    xTaskCreate(ack_monitor_task, "ack_monitor", 4096, NULL, 4, NULL);
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 3, NULL);
    xTaskCreate(discovery_task, "discovery", 3072, NULL, 2, NULL);
    
    ESP_LOGI(TAG, "Master controller ready");
    ESP_LOGI(TAG, "Commands: takeoff, land, forward, backward, left, right, up, down, stop, speed [val], setid [new_id], discover");
    ESP_LOGI(TAG, "Format: COMMAND [DRONE_ID] [PARAM1] [PARAM2]");
    ESP_LOGI(TAG, "Example: forward 1 1500 0  (drone 1 forward at speed 1500)");
    ESP_LOGI(TAG, "Example: speed 0 2000 0   (all drones speed 2000)");
}
