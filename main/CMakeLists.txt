idf_component_register(SRCS 
    "main.c"
    "opendroneid.c"
    "wifi.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
)