// master_serial_forward.c
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define TAG "MASTER"
#define UART_NUM UART_NUM_0   // используем USB UART (0)
#define BUF_SIZE 128
#define BROADCAST_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

static uint8_t broadcast_mac[] = BROADCAST_MAC;

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

static void espnow_master_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

    // Добавляем широковещательный пир (без шифрования)
    esp_now_peer_info_t peer = {
        .peer_addr = broadcast_mac,
        .channel = 0,
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_LOGI(TAG, "ESP-NOW master ready");
}

static void forward_task(void *arg) {
    uint8_t buffer[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buffer[len] = '\0';
            // Удаляем перевод строки, если есть
            if (buffer[len-1] == '\n') buffer[len-1] = '\0';
            if (buffer[len-2] == '\r') buffer[len-2] = '\0';
            ESP_LOGI(TAG, "Send: %s", buffer);
            esp_err_t err = esp_now_send(broadcast_mac, buffer, strlen((char*)buffer));
            if (err != ESP_OK) ESP_LOGE(TAG, "Send error %d", err);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    uart_init();
    espnow_master_init();
    xTaskCreate(forward_task, "uart_forward", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Master ready. Send commands over USB UART.");
}