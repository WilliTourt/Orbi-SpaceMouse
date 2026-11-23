idf_component_register(SRCS "esp_hid_gap.c" "tasks.cpp" "joystick.cpp" "key.cpp" "motion.cpp" "config.cpp" "hid.cpp" "pwr.cpp" "led.cpp" "sys.cpp" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_timer bt esp_hid nvs_flash tmag3001 ws2812 axp202)
