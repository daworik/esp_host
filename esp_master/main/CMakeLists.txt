# ESP-NOW Drone Hive Master Controller
idf_component_register(
    SRCS "master_drone_controller.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
    REQUIRES
        esp_wifi
        esp_netif
        nvs_flash
        esp_timer
        driver
        esp_http_server
        json
)
