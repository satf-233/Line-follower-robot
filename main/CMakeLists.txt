idf_component_register(SRCS "test_follow.c" "start.c" "follow_brain.c" "utils.c" "motor.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc led_strip)