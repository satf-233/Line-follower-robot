idf_component_register(SRCS "encoder.c" "avoid.c" "test_follow.c" "start.c" "follow_brain.c" "utils.c" "motor.c" "lcd.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi led_strip esp_timer)