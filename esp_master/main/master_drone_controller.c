// master_drone_controller.c
// ESP-NOW Drone Hive Controller with Web Interface, encryption, acknowledgments, and individual addressing
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "cJSON.h"

#define TAG "MASTER"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 256

// LED for status indication
#define LED_PIN GPIO_NUM_2

// WiFi AP Configuration
#define AP_SSID "DroneHive_AP"
#define AP_PASSWORD "dronehive123"
#define AP_CHANNEL 6

// ESP-NOW Encryption Key (16 bytes for PMK)
static const uint8_t esp_now_key[ESP_NOW_KEY_LEN] = {
    0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x7a, 0x8b,
    0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03
};

// Maximum number of drones
#define MAX_DRONES 10
#define DRONE_HEARTBEAT_TIMEOUT_MS 3000
#define COMMAND_RETRY_COUNT 3
#define COMMAND_TIMEOUT_MS 500

// MAC addresses of drones - EDIT THESE WITH YOUR DRONE MAC ADDRESSES
// Format: {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
// Find MAC addresses by flashing slave code and checking Serial Monitor output
static const uint8_t drone_macs[MAX_DRONES][6] = {
    {0xac, 0xeb, 0xe6, 0x55, 0xf1, 0xe4}
};
static uint8_t num_drones_configured = 1;  // Set to number of drones you have (1-10)

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
    uint16_t uptime_sec;
    int16_t speed;
    char last_command[32];
} drone_t;

static drone_t drones[MAX_DRONES];
static uint8_t drone_count = 0;
static uint8_t current_seq = 0;
static QueueHandle_t ack_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;
static httpd_handle_t server = NULL;

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

// Send data with optional encryption
static esp_err_t send_data(const uint8_t *mac, const void *data, size_t len, bool encrypt) {
    esp_now_peer_info_t peer = {
        .peer_addr = {0},
        .channel = AP_CHANNEL,
        .ifidx = ESP_IF_WIFI_AP,
        .encrypt = encrypt,
    };
    memcpy(peer.peer_addr, mac, 6);

     // Only set LMK if encryption is enabled
    if (encrypt) {
        memcpy(peer.lmk, esp_now_key, ESP_NOW_KEY_LEN);
    }
    
    // Check if peer exists, add if not
    if (!esp_now_is_peer_exist(mac)) {
        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer: %d", err);
            return err;
        }
    }
    
    return esp_now_send(mac, data, len);
}

static esp_err_t send_encrypted(const uint8_t *mac, const void *data, size_t len) {
    return send_data(mac, data, len, true);
}

// Send unencrypted broadcast
static esp_err_t send_broadcast(const void *data, size_t len) {
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (int i = 0; i < drone_count; i++) {
        if (!esp_now_is_peer_exist(drones[i].mac_addr)) {
            esp_now_peer_info_t peer = {
                .peer_addr = {0},
                .channel = AP_CHANNEL,
                .ifidx = ESP_IF_WIFI_AP,
                .encrypt = true,
            };
            memcpy(peer.peer_addr, drones[i].mac_addr, 6);
            memcpy(peer.lmk, esp_now_key, ESP_NOW_KEY_LEN);
            esp_err_t err = esp_now_add_peer(&peer);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to add peer for broadcast: %d", err);
            }
        }
    }
    return esp_now_send(broadcast_mac, data, len);
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
    
    
    if (drone_id == 0) {
        // Broadcast
        
        ESP_LOGI(TAG, "Broadcast command: %u, params=(%d, %d)", command, param1, param2);
    } else {
        // Individual drone
        int idx = find_drone_by_id(drone_id);
        if (idx < 0) {
            ESP_LOGE(TAG, "Drone ID %u not found", drone_id);
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGI(TAG, "Command to drone %u: %u, params=(%d, %d)", 
                 drone_id, command, param1, param2);
    }
    
    // Send with retries
    for (int retry = 0; retry < COMMAND_RETRY_COUNT; retry++) {
        esp_err_t err;
        if (drone_id == 0) {
             // Broadcast - send to all drones
            ESP_LOGD(TAG, "Sending broadcast (retry %d/%d)", retry + 1, COMMAND_RETRY_COUNT);

            if (num_drones_configured > 0 && drone_count == 0) {
                for (int i = 0; i < num_drones_configured; i++) {
                    add_drone(drone_macs[i], i + 1);
                }
            }

            err = send_broadcast(&msg, sizeof(msg));
        } else {
            // Individual drone
            int idx = find_drone_by_id(drone_id);
            if (idx < 0) {
                ESP_LOGE(TAG, "Drone ID %u not found", drone_id);
                return ESP_ERR_NOT_FOUND;
            }
            ESP_LOGD(TAG, "Sending to drone %u (retry %d/%d)", drone_id, retry + 1, COMMAND_RETRY_COUNT);
            err = send_encrypted(drones[idx].mac_addr, &msg, sizeof(msg));
        }
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Send successful");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "Send failed with error %d (retry %d/%d)", err, retry + 1, COMMAND_RETRY_COUNT);
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
    
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk(esp_now_key));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    ESP_LOGI(TAG, "ESP-NOW master initialized with encryption on channel %d", AP_CHANNEL);
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

// CORS headers helper
static esp_err_t cors(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    return ESP_OK;
}

// API: Get drone status
static esp_err_t api_status_handler(httpd_req_t *req) {
    cors(req);
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
    cJSON *root = cJSON_CreateObject();
    cJSON *drones_arr = cJSON_CreateArray();
    
    uint32_t now = esp_timer_get_time() / 1000;
    
    for (int i = 0; i < drone_count; i++) {
        cJSON *drone_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(drone_obj, "id", drones[i].id);
        
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 drones[i].mac_addr[0], drones[i].mac_addr[1], drones[i].mac_addr[2],
                 drones[i].mac_addr[3], drones[i].mac_addr[4], drones[i].mac_addr[5]);
        cJSON_AddStringToObject(drone_obj, "mac", mac_str);
        
        bool lost = (now - drones[i].last_heartbeat) > DRONE_HEARTBEAT_TIMEOUT_MS;
        cJSON_AddBoolToObject(drone_obj, "lost", lost);
        cJSON_AddStringToObject(drone_obj, "status", lost ? "LOST SIGNAL" : "OK");
        cJSON_AddNumberToObject(drone_obj, "rssi", drones[i].rssi);
        cJSON_AddNumberToObject(drone_obj, "uptime", drones[i].uptime_sec);
        cJSON_AddNumberToObject(drone_obj, "speed", drones[i].speed);
        cJSON_AddStringToObject(drone_obj, "last_cmd", drones[i].last_command);
        
        cJSON_AddItemToArray(drones_arr, drone_obj);
    }
    
    cJSON_AddItemToObject(root, "drones", drones_arr);
    cJSON_AddNumberToObject(root, "count", drone_count);
    
    char *resp = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    free(resp);
    return ESP_OK;
}

// API: Send command to drones
static esp_err_t api_command_handler(httpd_req_t *req) {
    if (req->method == HTTP_OPTIONS) {
        cors(req);
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    cors(req);
    
    if (req->method != HTTP_POST) {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method not allowed");
        return ESP_FAIL;
    }
    
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty request");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *cmd_item = cJSON_GetObjectItem(json, "command");
    cJSON *drone_id_item = cJSON_GetObjectItem(json, "drone_id");
    cJSON *param1_item = cJSON_GetObjectItem(json, "param1");
    cJSON *param2_item = cJSON_GetObjectItem(json, "param2");
    
    if (!cJSON_IsString(cmd_item)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing command");
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    uint8_t drone_id = 0;
    if (cJSON_IsNumber(drone_id_item)) {
        drone_id = (uint8_t)drone_id_item->valueint;
    }
    
    int16_t param1 = cJSON_IsNumber(param1_item) ? (int16_t)param1_item->valueint : 0;
    int16_t param2 = cJSON_IsNumber(param2_item) ? (int16_t)param2_item->valueint : 0;
    
    const char *cmd = cmd_item->valuestring;
    uint8_t command = CMD_STOP;
    
    if (strcmp(cmd, "forward") == 0) command = CMD_FORWARD;
    else if (strcmp(cmd, "backward") == 0) command = CMD_BACKWARD;
    else if (strcmp(cmd, "stop") == 0) command = CMD_STOP;
    else if (strcmp(cmd, "left") == 0) command = CMD_LEFT;
    else if (strcmp(cmd, "right") == 0) command = CMD_RIGHT;
    else if (strcmp(cmd, "speed") == 0) command = CMD_SET_SPEED;
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown command");
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    // Update last_command for display
    for (int i = 0; i < drone_count; i++) {
        if (drone_id == 0 || drones[i].id == drone_id) {
            snprintf(drones[i].last_command, sizeof(drones[i].last_command), "%s", cmd);
        }
    }
    
    esp_err_t err = send_command(drone_id, command, param1, param2);
    cJSON_Delete(json);
    
    if (err == ESP_OK) {
        httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send");
    }
    return err;
}

// Web page handler
static const char webpage_html[] = R"rawliteral(<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Drone Hive Control</title>
<style>
body{font-family:Arial;background:#f0f0f0;padding:20px;text-align:center}
.container{max-width:800px;margin:auto;background:#fff;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}
h1{color:#333}
.status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin:20px 0}
.drone-card{background:#f8f9fa;padding:15px;border-radius:8px;border-left:4px solid #28a745}
.drone-card.lost{border-left-color:#dc3545;background:#fff5f5}
.btn-group{margin:20px 0}
button{padding:15px 30px;margin:5px;font-size:16px;border:none;border-radius:5px;cursor:pointer;color:#fff}
.btn-fwd{background:#28a745}.btn-bwd{background:#dc3545}.btn-stop{background:#6c757d}
.btn-left{background:#17a2b8}.btn-right{background:#17a2b8}
select{padding:10px;font-size:16px;margin:10px}
.error{color:#dc3545;font-weight:bold}
.ok{color:#28a745}
</style>
</head>
<body>
<div class='container'>
<h1>🚁 Drone Hive Control</h1>
<p>Connect to WiFi: <b>DroneHive_AP</b> (password: dronehive123)</p>

<h3>Drone Status</h3>
<div class='status-grid' id='droneStatus'>Loading...</div>

<h3>Send Commands</h3>
<select id='droneSelect'>
<option value='0'>All Drones</option>
</select>
<br>
<div class='btn-group'>
<button class='btn-fwd' ontouchstart='sendCmd("forward")' onmousedown='sendCmd("forward")'>⬆ FORWARD</button><br>
<button class='btn-left' ontouchstart='sendCmd("left")' onmousedown='sendCmd("left")'>⬅ LEFT</button>
<button class='btn-stop' ontouchstart='sendCmd("stop")' onmousedown='sendCmd("stop")'>⏹ STOP</button>
<button class='btn-right' ontouchstart='sendCmd("right")' onmousedown='sendCmd("right")'>RIGHT ➡</button><br>
<button class='btn-bwd' ontouchstart='sendCmd("backward")' onmousedown='sendCmd("backward")'>⬇ BACKWARD</button>
</div>
<p id='statusMsg'></p>
</div>

<script>
let drones=[];

function loadStatus(){
    fetch('/api/status').then(r=>r.json()).then(d=>{
        drones=d.drones||[];
        updateStatusDisplay(d);
        updateDroneSelect(d);
    }).catch(e=>console.error('Status error:',e));
}

function updateStatusDisplay(data){
    let html='';
    (data.drones||[]).forEach(d=>{
        let cls=d.lost?'drone-card lost':'drone-card';
        html+=`<div class="${cls}">
            <strong>Drone ${d.id}</strong><br>
            MAC: ${d.mac}<br>
            Status: <span class="${d.lost?'error':'ok'}">${d.status}</span><br>
            RSSI: ${d.rssi} dBm<br>
            Uptime: ${d.uptime}s<br>
            Speed: ${d.speed}<br>
            Last Cmd: ${d.last_cmd||'-'}
        </div>`;
    });
    if(html==='')html='<p>No drones connected yet.</p>';
    document.getElementById('droneStatus').innerHTML=html;
}

function updateDroneSelect(data){
    let sel=document.getElementById('droneSelect');
    let cur=sel.value;
    sel.innerHTML='<option value="0">All Drones</option>';
    (data.drones||[]).forEach(d=>{
        sel.innerHTML+=`<option value="${d.id}">Drone ${d.id}</option>`;
    });
    if(cur!=='0')sel.value=cur;
}

function sendCmd(cmd){
    let droneId=parseInt(document.getElementById('droneSelect').value)||0;
    let msg=document.getElementById('statusMsg');
    msg.textContent='Sending...';
    msg.className='';
    
    fetch('/api/command',{
        method:'POST',
        headers:{'Content-Type':'application/json'},
        body:JSON.stringify({command:cmd,drone_id:droneId,param1:1500,param2:0})
    })
    .then(r=>r.json())
    .then(d=>{
        if(d.ok){msg.textContent='Command sent!';msg.className='ok';}
        else{msg.textContent='Error!';msg.className='error';}
    })
    .catch(e=>{msg.textContent='Error: '+e;msg.className='error';});
}

setInterval(loadStatus,2000);
loadStatus();
</script>
</body>
</html>)rawliteral";

static esp_err_t webpage_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, webpage_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Initialize WiFi AP and HTTP server
static void wifi_ap_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi AP...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    wifi_config_t wc = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASSWORD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started: %s", AP_SSID);
    
    // Blink LED to indicate AP ready
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_cfg);
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void start_http_server(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;
    
    if (httpd_start(&server, &cfg) == ESP_OK) {
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri = "/", .method = HTTP_GET, .handler = webpage_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri = "/api/command", .method = HTTP_POST, .handler = api_command_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri = "/api/command", .method = HTTP_OPTIONS, .handler = api_command_handler});
        httpd_register_uri_handler(server, &(httpd_uri_t){.uri = "/api/status", .method = HTTP_OPTIONS, .handler = api_status_handler});
        ESP_LOGI(TAG, "HTTP server started on http://192.168.4.1");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}

// Heartbeat task - sends periodic heartbeats to all drones
static void heartbeat_task(void *arg) {
    while (1) {
        // Send broadcast heartbeat to keep connections alive
        send_command(0, CMD_STOP, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));  // Every 500ms
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
    
    // Initialize WiFi AP first (needed for ESP-NOW in AP mode)
    wifi_ap_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Initialize ESP-NOW
    espnow_master_init();
    
    // Register configured drones from MAC array
    if (num_drones_configured > 0 && num_drones_configured <= MAX_DRONES) {
        for (int i = 0; i < num_drones_configured; i++) {
            add_drone(drone_macs[i], i + 1);
        }
        ESP_LOGI(TAG, "Registered %u pre-configured drones", num_drones_configured);
    }
    
    // Create tasks
    xTaskCreate(command_parser_task, "cmd_parser", 4096, NULL, 5, NULL);
    xTaskCreate(ack_monitor_task, "ack_monitor", 4096, NULL, 4, NULL);
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 3, NULL);
    xTaskCreate(discovery_task, "discovery", 3072, NULL, 2, NULL);
    xTaskCreate(heartbeat_task, "heartbeat", 3072, NULL, 3, NULL);
    
    // Start HTTP server
    vTaskDelay(pdMS_TO_TICKS(500));
    start_http_server();
    
    ESP_LOGI(TAG, "Master controller ready");
    ESP_LOGI(TAG, "Web interface: http://192.168.4.1");
    ESP_LOGI(TAG, "Commands: forward, backward, left, right, stop, speed [val]");
    ESP_LOGI(TAG, "Format: COMMAND [DRONE_ID] [PARAM1] [PARAM2]");
    ESP_LOGI(TAG, "Example: forward 1 1500 0  (drone 1 forward at speed 1500)");
}
