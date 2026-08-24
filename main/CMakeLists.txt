idf_component_register(SRCS "utils.c" "motor.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio led_strip)