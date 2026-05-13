# ESP-NOW Drone Hive Master Controller
idf_component_register(
    SRCS "master_drone_controller.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
        esp_wifi
        nvs_flash
        esp_timer
        driver
)
