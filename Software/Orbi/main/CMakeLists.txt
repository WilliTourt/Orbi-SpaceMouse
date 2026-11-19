idf_component_register(SRCS "tasks.cpp" "joystick.cpp" "key.cpp" "motion.cpp" "config.cpp" "hid.cpp" "pwr.cpp" "led.cpp" "sys.cpp" "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_timer nvs_flash tmag3001 ws2812 axp202)
