# ESP-NOW Drone Hive Agent Controller
idf_component_register(
    SRCS "slave_drone_agent.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES 
        esp_wifi
        nvs_flash
        esp_timer
        driver
)
