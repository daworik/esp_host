// slave_motor.c
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
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

typedef struct {
    int step_pin;
    int dir_pin;
    volatile int32_t target_speed;
    volatile int32_t current_speed;
    volatile uint32_t steps;
    volatile bool step_state;
    gptimer_handle_t timer;
    bool invert_dir;   // для второго мотора если нужно
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

// --- Таймер ---
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

// --- Задача управления мотором (плавный разгон) ---
static void motor_control_task(void *arg) {
    motor_t *m = (motor_t*) arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);
    while (1) {
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
        vTaskDelayUntil(&last_wake, frequency);
    }
}

// --- Обработчик ESP-NOW (получение команд) ---
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    char cmd_buf[64];
    if (data_len < sizeof(cmd_buf)) {
        memcpy(cmd_buf, data, data_len);
        cmd_buf[data_len] = '\0';
        ESP_LOGI(TAG, "CMD: %s", cmd_buf);

        cJSON *json = cJSON_Parse(cmd_buf);
        if (!json) {
            ESP_LOGE(TAG, "Invalid JSON");
            return;
        }
        cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
        if (cJSON_IsString(cmd)) {
            const char *c = cmd->valuestring;
            if (strcmp(c, "forward") == 0) motor.target_speed = DEFAULT_SPEED;
            else if (strcmp(c, "backward") == 0) motor.target_speed = -DEFAULT_SPEED;
            else if (strcmp(c, "stop") == 0) motor.target_speed = 0;
        }
        cJSON_Delete(json);
    }
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
    ESP_LOGI(TAG, "ESP-NOW slave ready");
}

void app_main(void) {
    ESP_LOGI(TAG, "Slave starting");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    gpio_init_motor();
    motor_timer_init();
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, &motor, 5, NULL, 0);
    espnow_slave_init();
    ESP_LOGI(TAG, "Ready");
}