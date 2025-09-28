idf_component_register(SRCS "dshot_main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES DShotRMT esp_wifi esp_http_server nvs_flash esp_netif driver) 