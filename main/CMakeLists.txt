idf_component_register(SRCS "test_follow.c" "start.c" "follow_brain.c" "utils.c" "motor.c" "lcd.c" "avoid.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_timer led_strip)