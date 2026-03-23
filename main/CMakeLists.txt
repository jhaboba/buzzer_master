idf_component_register(SRCS "main_master.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio esp_driver_i2s
                    INCLUDE_DIRS ".")
