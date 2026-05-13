// slave_drone_agent.c
// ESP-NOW Drone Agent with encryption, acknowledgments, and telemetry
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#define TAG                 "DRONE_AGENT"

// Motor configuration
#define STEP_PIN            GPIO_NUM_5
#define DIR_PIN             GPIO_NUM_6
#define MS1_PIN             GPIO_NUM_12
#define MS2_PIN             GPIO_NUM_10
#define MAX_PPS             2000
#define ACCELERATION        500
#define DEFAULT_SPEED       1500

// ADC for battery monitoring
#define ADC_PIN             GPIO_NUM_4
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CHANNEL         ADC_CHANNEL_4

// ESP-NOW Encryption Key (MUST match master)
static const uint8_t esp_now_key[ESP_NOW_KEY_LEN] = {
    0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f, 0x7a, 0x8b,
    0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03,
    0x14, 0x25, 0x36, 0x47, 0x58, 0x69, 0x7a, 0x8b,
    0x9c, 0xad, 0xbe, 0xcf, 0xd0, 0xe1, 0xf2, 0x03
};

// Message types (must match master)
typedef enum {
    MSG_TYPE_COMMAND = 0,
    MSG_TYPE_ACK = 1,
    MSG_TYPE_TELEMETRY = 2,
    MSG_TYPE_DISCOVERY = 3,
    MSG_TYPE_DISCOVERY_RESPONSE = 4
} msg_type_t;

// Command types (must match master)
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

// Packed message structure (must match master)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t drone_id;
    uint8_t command;
    uint8_t seq_num;
    int16_t param1;
    int16_t param2;
    uint32_t timestamp;
} drone_message_t;

// Telemetry structure (must match master)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t drone_id;
    uint16_t battery_mv;
    int16_t speed;
    uint16_t uptime_sec;
    int8_t rssi;
    uint8_t status_flags;
    uint32_t timestamp;
} telemetry_message_t;

// Motor state
typedef struct {
    int step_pin;
    int dir_pin;
    volatile int32_t target_speed;
    volatile int32_t current_speed;
    volatile uint32_t steps;
    volatile bool step_state;
    gptimer_handle_t timer;
    bool invert_dir;
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
};

// Global state
static uint8_t my_drone_id = 1;  // Default ID, can be changed
static uint32_t start_time = 0;
static int8_t last_rssi = -100;
static uint8_t status_flags = 0;
static esp_now_peer_info_t master_peer;
static bool master_known = false;

// Queue for sending telemetry
static QueueHandle_t telemetry_queue = NULL;

// GPIO initialization
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

// Timer callback for step generation
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

// Initialize motor timer - called ONCE at startup
static void motor_timer_init(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz = 1 us resolution
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &motor.timer));
    
    gptimer_event_callbacks_t cbs = {.on_alarm = timer_callback};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(motor.timer, &cbs, &motor));
    
    ESP_ERROR_CHECK(gptimer_enable(motor.timer));
    ESP_LOGI(TAG, "Motor timer initialized");
}

// Update motor speed - only changes period, doesn't reconfigure timer
static void update_motor_speed(int32_t speed) {
    if (speed > MAX_PPS) speed = MAX_PPS;
    if (speed < -MAX_PPS) speed = -MAX_PPS;
    
    motor.target_speed = speed;
}

// Motor control task with proper acceleration
static void motor_control_task(void *arg) {
    motor_t *m = (motor_t*) arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);  // 20 Hz update rate
    
    while (1) {
        int32_t diff = m->target_speed - m->current_speed;
        
        if (diff != 0) {
            int32_t delta = (ACCELERATION * 50) / 1000;
            if (delta < 1) delta = 1;
            
            if (diff > 0) {
                m->current_speed += delta;
                if (m->current_speed > m->target_speed) 
                    m->current_speed = m->target_speed;
            } else {
                m->current_speed -= delta;
                if (m->current_speed < m->target_speed) 
                    m->current_speed = m->target_speed;
            }
        }
        
        int32_t s = m->current_speed;
        
        if (s != 0) {
            // Set direction
            int dir_level = (s > 0) ? 1 : 0;
            if (m->invert_dir) dir_level = !dir_level;
            gpio_set_level(m->dir_pin, dir_level);
            
            // Calculate period in microseconds
            uint32_t period_us = 1000000 / abs(s);
            if (period_us < 50) period_us = 50;  // Max frequency limit
            
            // Only update alarm config, don't reconfigure entire timer
            gptimer_alarm_config_t alarm_config = {
                .reload_count = 0,
                .alarm_count = period_us / 2,  // 50% duty cycle
                .flags.auto_reload_on_alarm = true,
            };
            gptimer_set_alarm_action(m->timer, &alarm_config);
            
            // Start if not running
            gptimer_start(m->timer);
        } else {
            gptimer_stop(m->timer);
            gpio_set_level(m->step_pin, 0);
        }
        
        vTaskDelayUntil(&last_wake, frequency);
    }
}

// Read battery voltage
static uint16_t read_battery_voltage(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc1_get_raw(ADC_CHANNEL);
    }
    uint32_t avg = sum / 8;
    
    // Convert to mV (adjust calibration for your setup)
    // With 12dB attenuation: 0-3.3V maps to 0-4095
    // Assuming voltage divider (e.g., 2:1 for up to 8.4V battery)
    uint32_t mv = (avg * 3300) / 4095;
    mv = mv * 2;  // Voltage divider ratio
    
    return (uint16_t)mv;
}

// Send acknowledgment to master
static void send_ack(const uint8_t *sender_mac, uint8_t seq_num) {
    if (!master_known) {
        memcpy(master_peer.peer_addr, sender_mac, 6);
        master_known = true;
    }
    
    drone_message_t ack;
    ack.msg_type = MSG_TYPE_ACK;
    ack.drone_id = my_drone_id;
    ack.command = 0;
    ack.seq_num = seq_num;
    ack.param1 = 0;
    ack.param2 = 0;
    ack.timestamp = esp_timer_get_time() / 1000;
    
    esp_now_send(sender_mac, (uint8_t*)&ack, sizeof(ack));
}

// Send telemetry to master
static void send_telemetry(void) {
    if (!master_known) {
        return;
    }
    
    telemetry_message_t telemetry;
    telemetry.msg_type = MSG_TYPE_TELEMETRY;
    telemetry.drone_id = my_drone_id;
    telemetry.battery_mv = read_battery_voltage();
    telemetry.speed = motor.current_speed;
    telemetry.uptime_sec = (esp_timer_get_time() / 1000000) - start_time;
    telemetry.rssi = last_rssi;
    telemetry.status_flags = status_flags;
    telemetry.timestamp = esp_timer_get_time() / 1000;
    
    esp_now_send(master_peer.peer_addr, (uint8_t*)&telemetry, sizeof(telemetry));
}

// Send discovery response
static void send_discovery_response(const uint8_t *sender_mac) {
    drone_message_t response;
    response.msg_type = MSG_TYPE_DISCOVERY_RESPONSE;
    response.drone_id = my_drone_id;
    response.command = 0;
    response.seq_num = 0;
    response.param1 = 0;
    response.param2 = 0;
    response.timestamp = esp_timer_get_time() / 1000;
    
    esp_now_send(sender_mac, (uint8_t*)&response, sizeof(response));
    
    if (!master_known) {
        memcpy(master_peer.peer_addr, sender_mac, 6);
        master_known = true;
        ESP_LOGI(TAG, "Master registered");
    }
}

// Process received command
static void process_command(const uint8_t *sender_mac, const drone_message_t *msg) {
    // Register master on first contact
    if (!master_known) {
        memcpy(master_peer.peer_addr, sender_mac, 6);
        master_known = true;
        ESP_LOGI(TAG, "Master registered from MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 sender_mac[0], sender_mac[1], sender_mac[2],
                 sender_mac[3], sender_mac[4], sender_mac[5]);
    }
    
    // Check if message is for us (or broadcast)
    if (msg->drone_id != 0 && msg->drone_id != my_drone_id) {
        return;  // Not for this drone
    }
    
    ESP_LOGI(TAG, "Command: %u, params=(%d, %d)", msg->command, msg->param1, msg->param2);
    
    // Execute command
    switch (msg->command) {
        case CMD_FORWARD:
            update_motor_speed(msg->param1 != 0 ? msg->param1 : DEFAULT_SPEED);
            break;
            
        case CMD_BACKWARD:
            update_motor_speed(msg->param1 != 0 ? -msg->param1 : -DEFAULT_SPEED);
            break;
            
        case CMD_STOP:
            update_motor_speed(0);
            break;
            
        case CMD_SET_SPEED:
            update_motor_speed(msg->param1);
            break;
            
        case CMD_SET_ID:
            if (msg->param1 > 0 && msg->param1 <= 255) {
                my_drone_id = (uint8_t)msg->param1;
                ESP_LOGI(TAG, "ID changed to %u", my_drone_id);
            }
            break;
            
        case CMD_DISCOVER:
            send_discovery_response(sender_mac);
            break;
            
        case CMD_TAKEOFF:
        case CMD_LAND:
        case CMD_LEFT:
        case CMD_RIGHT:
        case CMD_UP:
        case CMD_DOWN:
            // For multi-rotor drones, these would control other motors
            ESP_LOGI(TAG, "Flight command received (not implemented for single motor)");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: %u", msg->command);
            break;
    }
    
    // Send acknowledgment
    send_ack(sender_mac, msg->seq_num);
}

// ESP-NOW receive callback
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    const uint8_t *mac = recv_info->src_addr;
    last_rssi = recv_info->rssi;
    
    if (data_len < sizeof(drone_message_t)) {
        ESP_LOGW(TAG, "Packet too small: %d", data_len);
        return;
    }
    
    drone_message_t msg;
    memcpy(&msg, data, sizeof(msg));
    
    if (msg.msg_type == MSG_TYPE_COMMAND) {
        process_command(mac, &msg);
    }
}

// Telemetry task - sends periodic updates
static void telemetry_task(void *arg) {
    const TickType_t interval = pdMS_TO_TICKS(1000);  // 1 second
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        send_telemetry();
        vTaskDelayUntil(&last_wake, interval);
    }
}

// Initialize ESP-NOW Slave
static void espnow_slave_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk(esp_now_key));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    // Configure peer for encrypted communication
    memset(&master_peer, 0, sizeof(master_peer));
    master_peer.channel = 0;
    master_peer.ifidx = ESP_IF_WIFI_STA;
    master_peer.encrypt = true;
    memcpy(master_peer.lmk, esp_now_key, ESP_NOW_KEY_LEN);
    
    start_time = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "ESP-NOW agent ready (ID=%u)", my_drone_id);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Initialize hardware
    gpio_init_motor();
    motor_timer_init();
    
    // Create motor control task on core 0
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, &motor, 5, NULL, 0);
    
    // Initialize ESP-NOW
    espnow_slave_init();
    
    // Create telemetry task
    xTaskCreate(telemetry_task, "telemetry", 3072, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "Drone agent started");
    ESP_LOGI(TAG, "Waiting for commands from master...");
}
