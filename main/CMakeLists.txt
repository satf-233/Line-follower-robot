idf_component_register(SRCS "utils.c" "motor.c" "servo.c" "lcd.c" "follow_brain.c" "avoid.c" "image.c" "encoder.c" "start.c" "test_follow.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_timer led_strip fatfs esp_jpeg)
