idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer esp_http_server esp_wifi nvs_flash esp_event cjson)
