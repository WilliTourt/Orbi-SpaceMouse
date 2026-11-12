idf_component_register(SRCS "motion.cpp" "config.cpp" "hid.cpp" "pwr.cpp" "led.cpp" "sys.cpp" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio tmag3001 ws2812 axp202)
