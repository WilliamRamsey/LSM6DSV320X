idf_component_register(
    SRCS "LSM6DSV320X.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_spi
        esp_event
        esp_netif
        esp_timer
        esp_wifi
        lwip
        nvs_flash
)
