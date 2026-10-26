idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio tmag3001 ws2812 axp202)
