idf_component_register(SRCS "start.c" "follow_brain.c" "utils.c" "motor.c" "main.c" "lcd.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi led_strip)