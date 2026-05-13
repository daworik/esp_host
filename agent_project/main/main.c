// slave_motor.c - Improved ESP-NOW Drone Agent
// Key improvements:
// 1. Unique MAC-based addressing for individual drone control
// 2. Acknowledgment system with status feedback
// 3. Enhanced JSON command structure with request IDs
// 4. Link-loss detection and safe shutdown
// 5. Extended command set (speed control, position queries, diagnostics)
// 6. Thread-safe motor state access
// 7. Watchdog timer for safety

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "cJSON.h"

#define TAG                 "MOTOR_SLAVE"
#define STEP_PIN            GPIO_NUM_5
#define DIR_PIN             GPIO_NUM_6
#define MS1_PIN             GPIO_NUM_12
#define MS2_PIN             GPIO_NUM_10
#define MAX_PPS             2000
#define ACCELERATION        500
#define DEFAULT_SPEED       1500
#define LINK_TIMEOUT_MS     2000  // Safety timeout for lost connection
#define ACK_RETRY_COUNT     3

typedef struct {
    int step_pin;
    int dir_pin;
    volatile int32_t target_speed;
    volatile int32_t current_speed;
    volatile uint32_t steps;
    volatile bool step_state;
    gptimer_handle_t timer;
    bool invert_dir;
    SemaphoreHandle_t mutex;
    volatile uint32_t last_command_time;
    volatile bool link_active;
} motor_t;

static motor_t motor = {
    .step_pin = STEP_PIN,
    .dir_pin = DIR_PIN,
    .target_speed = 0,
    .current_speed = 0,
    .steps = 0,
    .step_state = false,
    .timer = NULL,
    .invert_dir = false,
    .mutex = NULL,
    .last_command_time = 0,
    .link_active = false,
};

static uint8_t host_mac[6] = {0};  // Store host MAC for targeted ACKs
static bool host_mac_set = false;

// --- GPIO ---
static void gpio_init_motor(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << motor.step_pin) | (1ULL << motor.dir_pin) |
                        (1ULL << MS1_PIN) | (1ULL << MS2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cfg);
    gpio_set_level(motor.dir_pin, 1);
    gpio_set_level(motor.step_pin, 0);
    gpio_set_level(MS1_PIN, 0);
    gpio_set_level(MS2_PIN, 0);
    ESP_LOGI(TAG, "GPIO initialized");
}

// --- Timer ISR ---
static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    motor_t *m = (motor_t*) user_ctx;
    if (m->current_speed == 0) {
        gpio_set_level(m->step_pin, 0);
        return false;
    }
    m->step_state = !m->step_state;
    gpio_set_level(m->step_pin, m->step_state ? 1 : 0);
    if (m->step_state) m->steps++;
    return true;
}

static void motor_timer_init(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &motor.timer));
    gptimer_event_callbacks_t cbs = {.on_alarm = timer_callback};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(motor.timer, &cbs, &motor));
    ESP_ERROR_CHECK(gptimer_enable(motor.timer));
}

// --- Motor Control Task with Safety Monitoring ---
static void motor_control_task(void *arg) {
    motor_t *m = (motor_t*) arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);
    
    while (1) {
        // Check link timeout
        uint32_t now = esp_timer_get_time() / 1000;
        if (m->link_active && (now - m->last_command_time > LINK_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "Link lost! Emergency stop");
            m->link_active = false;
            xSemaphoreTake(m->mutex, portMAX_DELAY);
            m->target_speed = 0;
            xSemaphoreGive(m->mutex);
        }

        xSemaphoreTake(m->mutex, portMAX_DELAY);
        int32_t diff = m->target_speed - m->current_speed;
        if (diff > 0) {
            int32_t inc = (ACCELERATION * 50) / 1000;
            if (inc < 1) inc = 1;
            m->current_speed += inc;
            if (m->current_speed > m->target_speed) m->current_speed = m->target_speed;
        } else if (diff < 0) {
            int32_t dec = (ACCELERATION * 50) / 1000;
            if (dec < 1) dec = 1;
            m->current_speed -= dec;
            if (m->current_speed < m->target_speed) m->current_speed = m->target_speed;
        }
        if (m->current_speed > MAX_PPS) m->current_speed = MAX_PPS;
        if (m->current_speed < -MAX_PPS) m->current_speed = -MAX_PPS;

        int32_t s = m->current_speed;
        if (s != 0) {
            int dir_level = (s > 0) ? 1 : 0;
            if (m->invert_dir) dir_level = !dir_level;
            gpio_set_level(m->dir_pin, dir_level);
            uint32_t period_us = 1000000 / abs(s);
            if (period_us < 50) period_us = 50;
            gptimer_alarm_config_t alarm_config = {
                .reload_count = 0,
                .alarm_count = period_us / 2,
                .flags.auto_reload_on_alarm = true,
            };
            gptimer_set_alarm_action(m->timer, &alarm_config);
            gptimer_start(m->timer);
        } else {
            gptimer_stop(m->timer);
            gpio_set_level(m->step_pin, 0);
        }
        xSemaphoreGive(m->mutex);
        
        vTaskDelayUntil(&last_wake, frequency);
    }
}

// --- Send ACK to Host ---
static void send_ack(const uint8_t *src_mac, int32_t request_id, const char *status, int32_t data_value) {
    if (!host_mac_set) return;
    
    cJSON *ack = cJSON_CreateObject();
    cJSON_AddStringToObject(ack, "type", "ack");
    cJSON_AddNumberToObject(ack, "req_id", request_id);
    cJSON_AddStringToObject(ack, "status", status);
    cJSON_AddNumberToObject(ack, "steps", motor.steps);
    cJSON_AddNumberToObject(ack, "speed", motor.current_speed);
    if (data_value != INT32_MAX) {
        cJSON_AddNumberToObject(ack, "value", data_value);
    }
    
    char *json_str = cJSON_PrintUnformatted(ack);
    if (json_str) {
        esp_err_t err = esp_now_send(host_mac, (uint8_t*)json_str, strlen(json_str));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ACK send failed: %d", err);
        }
        free(json_str);
    }
    cJSON_Delete(ack);
}

// --- ESP-NOW Receive Callback with Enhanced Command Handling ---
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    // Store host MAC from first received packet
    if (!host_mac_set) {
        memcpy(host_mac, recv_info->src_addr, 6);
        host_mac_set = true;
        ESP_LOGI(TAG, "Host MAC: %02x:%02x:%02x:%02x:%02x:%02x", 
                 host_mac[0], host_mac[1], host_mac[2], 
                 host_mac[3], host_mac[4], host_mac[5]);
    }
    
    motor.last_command_time = esp_timer_get_time() / 1000;
    motor.link_active = true;
    
    char cmd_buf[128];
    if (data_len >= sizeof(cmd_buf)) {
        ESP_LOGW(TAG, "Packet too large: %d", data_len);
        return;
    }
    
    memcpy(cmd_buf, data, data_len);
    cmd_buf[data_len] = '\0';
    ESP_LOGD(TAG, "RX: %s", cmd_buf);

    cJSON *json = cJSON_Parse(cmd_buf);
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON");
        send_ack(recv_info->src_addr, -1, "parse_error", INT32_MAX);
        return;
    }
    
    cJSON *type = cJSON_GetObjectItem(json, "type");
    if (!cJSON_IsString(type)) {
        cJSON_Delete(json);
        return;
    }
    
    if (strcmp(type->valuestring, "cmd") == 0) {
        cJSON *req_id_obj = cJSON_GetObjectItem(json, "req_id");
        int32_t req_id = cJSON_IsNumber(req_id_obj) ? (int32_t)req_id_obj->valuedouble : -1;
        
        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
        if (cJSON_IsString(cmd)) {
            const char *c = cmd->valuestring;
            xSemaphoreTake(motor.mutex, portMAX_DELAY);
            
            if (strcmp(c, "forward") == 0) {
                motor.target_speed = DEFAULT_SPEED;
                ESP_LOGI(TAG, "Command: FORWARD");
                send_ack(recv_info->src_addr, req_id, "ok", INT32_MAX);
            } else if (strcmp(c, "backward") == 0) {
                motor.target_speed = -DEFAULT_SPEED;
                ESP_LOGI(TAG, "Command: BACKWARD");
                send_ack(recv_info->src_addr, req_id, "ok", INT32_MAX);
            } else if (strcmp(c, "stop") == 0) {
                motor.target_speed = 0;
                ESP_LOGI(TAG, "Command: STOP");
                send_ack(recv_info->src_addr, req_id, "ok", INT32_MAX);
            } else if (strcmp(c, "set_speed") == 0) {
                cJSON *speed_obj = cJSON_GetObjectItem(json, "speed");
                if (cJSON_IsNumber(speed_obj)) {
                    int32_t speed = (int32_t)speed_obj->valuedouble;
                    if (speed > MAX_PPS) speed = MAX_PPS;
                    if (speed < -MAX_PPS) speed = -MAX_PPS;
                    motor.target_speed = speed;
                    ESP_LOGI(TAG, "Command: SET_SPEED=%d", speed);
                    send_ack(recv_info->src_addr, req_id, "ok", speed);
                } else {
                    send_ack(recv_info->src_addr, req_id, "invalid_param", INT32_MAX);
                }
            } else if (strcmp(c, "query_status") == 0) {
                ESP_LOGI(TAG, "Command: QUERY_STATUS");
                send_ack(recv_info->src_addr, req_id, "status", motor.current_speed);
            } else if (strcmp(c, "reset_steps") == 0) {
                motor.steps = 0;
                ESP_LOGI(TAG, "Command: RESET_STEPS");
                send_ack(recv_info->src_addr, req_id, "ok", 0);
            } else {
                ESP_LOGW(TAG, "Unknown command: %s", c);
                send_ack(recv_info->src_addr, req_id, "unknown_cmd", INT32_MAX);
            }
            
            xSemaphoreGive(motor.mutex);
        } else {
            send_ack(recv_info->src_addr, -1, "missing_cmd", INT32_MAX);
        }
    }
    
    cJSON_Delete(json);
}

static void espnow_slave_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    // Set power for better range
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)"YOUR_PMK_KEY_16B"));  // Optional encryption
    ESP_LOGI(TAG, "ESP-NOW slave ready");
}

void app_main(void) {
    ESP_LOGI(TAG, "Slave starting");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    motor.mutex = xSemaphoreCreateMutex();
    if (motor.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    gpio_init_motor();
    motor_timer_init();
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, &motor, 5, NULL, 0);
    espnow_slave_init();
    ESP_LOGI(TAG, "Ready - MAC: " MACSTR, MAC2STR(esp_efuse_mac_get_default()));
}
