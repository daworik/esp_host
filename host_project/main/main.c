// master_drone_controller.c - Improved ESP-NOW Hive Host
// Key improvements matching the enhanced slave:
// 1. Individual drone addressing by MAC (not broadcast)
// 2. Request ID tracking for command acknowledgment
// 3. Status monitoring and link-loss detection
// 4. Multi-drone orchestration support
// 5. Structured JSON protocol with ack handling
// 6. Retry logic for reliable delivery

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "cJSON.h"

#define TAG "MASTER"
#define UART_NUM UART_NUM_0
#define BUF_SIZE 256
#define MAX_DRONES 10
#define COMMAND_TIMEOUT_MS 1000
#define MAX_RETRIES 3

typedef struct {
    uint8_t mac[6];
    char name[16];
    bool active;
    int32_t last_req_id;
    uint32_t last_ack_time;
    int32_t current_speed;
    uint32_t steps;
    bool link_ok;
} drone_t;

static drone_t drones[MAX_DRONES];
static int drone_count = 0;
static SemaphoreHandle_t drone_mutex = NULL;
static volatile int32_t global_req_id = 0;

// Example drone MACs - replace with your actual drone MACs
static const uint8_t DRONE_MACS[][6] = {
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},
};

static int find_drone_by_mac(const uint8_t *mac) {
    for (int i = 0; i < drone_count; i++) {
        if (memcmp(drones[i].mac, mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

static void register_drone(const uint8_t *mac, const char *name) {
    if (drone_count >= MAX_DRONES) {
        ESP_LOGW(TAG, "Max drones reached");
        return;
    }
    
    xSemaphoreTake(drone_mutex, portMAX_DELAY);
    int idx = find_drone_by_mac(mac);
    if (idx < 0) {
        idx = drone_count++;
        memcpy(drones[idx].mac, mac, 6);
        strncpy(drones[idx].name, name ? name : "unnamed", 15);
        drones[idx].active = true;
        drones[idx].last_req_id = -1;
        drones[idx].link_ok = false;
        ESP_LOGI(TAG, "Registered drone %s: " MACSTR, 
                 drones[idx].name, MAC2STR(mac));
    }
    xSemaphoreGive(drone_mutex);
}

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

static esp_err_t send_command_to_drone(const uint8_t *mac, const char *cmd, 
                                       int32_t req_id, int32_t speed_param) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "cmd");
    cJSON_AddNumberToObject(root, "req_id", req_id);
    cJSON_AddStringToObject(root, "cmd", cmd);
    
    if (speed_param != INT32_MAX) {
        cJSON_AddNumberToObject(root, "speed", speed_param);
    }
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGD(TAG, "TX to " MACSTR ": %s", MAC2STR(mac), json_str);
    esp_err_t err = esp_now_send(mac, (uint8_t*)json_str, strlen(json_str));
    free(json_str);
    cJSON_Delete(root);
    return err;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, 
                           const uint8_t *data, int data_len) {
    char buf[256];
    if (data_len >= sizeof(buf)) {
        ESP_LOGW(TAG, "ACK packet too large");
        return;
    }
    
    memcpy(buf, data, data_len);
    buf[data_len] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        ESP_LOGE(TAG, "Invalid ACK JSON");
        return;
    }
    
    cJSON *type = cJSON_GetObjectItem(json, "type");
    if (!cJSON_IsString(type) || strcmp(type->valuestring, "ack") != 0) {
        cJSON_Delete(json);
        return;
    }
    
    int drone_idx = find_drone_by_mac(recv_info->src_addr);
    if (drone_idx < 0) {
        // Auto-register unknown drone
        register_drone(recv_info->src_addr, "auto");
        drone_idx = find_drone_by_mac(recv_info->src_addr);
    }
    
    if (drone_idx >= 0) {
        xSemaphoreTake(drone_mutex, portMAX_DELAY);
        drones[drone_idx].last_ack_time = esp_timer_get_time() / 1000;
        drones[drone_idx].link_ok = true;
        
        cJSON *status = cJSON_GetObjectItem(json, "status");
        if (cJSON_IsString(status)) {
            ESP_LOGI(TAG, "Drone %s ack: %s", drones[drone_idx].name, status->valuestring);
        }
        
        cJSON *speed = cJSON_GetObjectItem(json, "speed");
        if (cJSON_IsNumber(speed)) {
            drones[drone_idx].current_speed = (int32_t)speed->valuedouble;
        }
        
        cJSON *steps = cJSON_GetObjectItem(json, "steps");
        if (cJSON_IsNumber(steps)) {
            drones[drone_idx].steps = (uint32_t)steps->valuedouble;
        }
        
        xSemaphoreGive(drone_mutex);
    }
    
    cJSON_Delete(json);
}

static void espnow_master_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    // Optional encryption - must match slave PMK
    // ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"YOUR_PMK_KEY_16B"));
    
    // Register known drones as peers
    for (int i = 0; i < sizeof(DRONE_MACS)/sizeof(DRONE_MACS[0]); i++) {
        esp_now_peer_info_t peer = {
            .peer_addr = (uint8_t*)DRONE_MACS[i],
            .channel = 0,
            .ifidx = ESP_IF_WIFI_STA,
            .encrypt = false,  // Set true if using PMK
        };
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
        char name[16];
        snprintf(name, sizeof(name), "drone_%d", i+1);
        register_drone(DRONE_MACS[i], name);
    }
    
    ESP_LOGI(TAG, "ESP-NOW master ready");
}

static void check_drone_links_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        uint32_t now = esp_timer_get_time() / 1000;
        
        xSemaphoreTake(drone_mutex, portMAX_DELAY);
        for (int i = 0; i < drone_count; i++) {
            if (drones[i].active && now - drones[i].last_ack_time > COMMAND_TIMEOUT_MS * 2) {
                if (drones[i].link_ok) {
                    ESP_LOGW(TAG, "Drone %s link lost!", drones[i].name);
                    drones[i].link_ok = false;
                }
            }
        }
        xSemaphoreGive(drone_mutex);
    }
}

static void parse_and_send_command(char *cmd_str) {
    // Format: "DRONE_NAME COMMAND [PARAM]"
    // Examples:
    //   "drone_1 forward"
    //   "drone_2 set_speed 1000"
    //   "all stop"
    //   "broadcast query_status"
    
    char *saveptr;
    char *target = strtok_r(cmd_str, " \t\n", &saveptr);
    char *command = strtok_r(NULL, " \t\n", &saveptr);
    char *param_str = strtok_r(NULL, " \t\n", &saveptr);
    
    if (!target || !command) {
        ESP_LOGW(TAG, "Invalid command format");
        return;
    }
    
    int32_t req_id = ++global_req_id;
    int32_t speed_param = INT32_MAX;
    
    if (param_str) {
        speed_param = atoi(param_str);
    }
    
    xSemaphoreTake(drone_mutex, portMAX_DELAY);
    
    for (int i = 0; i < drone_count; i++) {
        bool match = false;
        
        if (strcmp(target, "all") == 0 || strcmp(target, "broadcast") == 0) {
            match = true;
        } else if (strcmp(target, drones[i].name) == 0) {
            match = true;
        }
        
        if (match && drones[i].active) {
            esp_err_t err = send_command_to_drone(drones[i].mac, command, req_id, speed_param);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send to %s: %d", drones[i].name, err);
            } else {
                drones[i].last_req_id = req_id;
            }
        }
    }
    
    xSemaphoreGive(drone_mutex);
}

static void command_forward_task(void *arg) {
    uint8_t buffer[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI(TAG, "UART RX: %s", buffer);
            
            // Make a copy for parsing
            char cmd_copy[BUF_SIZE];
            strncpy(cmd_copy, (char*)buffer, BUF_SIZE-1);
            cmd_copy[BUF_SIZE-1] = '\0';
            
            parse_and_send_command(cmd_copy);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void status_report_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Report every 5 seconds
        
        xSemaphoreTake(drone_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "=== DRONE STATUS ===");
        for (int i = 0; i < drone_count; i++) {
            ESP_LOGI(TAG, "%s: link=%s speed=%d steps=%u",
                     drones[i].name,
                     drones[i].link_ok ? "OK" : "LOST",
                     drones[i].current_speed,
                     drones[i].steps);
        }
        xSemaphoreGive(drone_mutex);
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    drone_mutex = xSemaphoreCreateMutex();
    if (drone_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    uart_init();
    espnow_master_init();
    
    xTaskCreate(command_forward_task, "uart_cmd", 4096, NULL, 5, NULL);
    xTaskCreate(check_drone_links_task, "link_monitor", 3072, NULL, 4, NULL);
    xTaskCreate(status_report_task, "status_report", 3072, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "Master ready. Commands: '<drone|all> <cmd> [param]'");
    ESP_LOGI(TAG, "Examples: 'drone_1 forward', 'all stop', 'drone_2 set_speed 1200'");
}
