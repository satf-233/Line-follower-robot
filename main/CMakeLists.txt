idf_component_register(SRCS "camera_audio.c" "servo.c" "test_cam.c" "test_audio.c" "start.c" "follow_brain.c" "utils.c" "motor.c" "lcd.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi led_strip fatfs esp_jpeg)

fatfs_create_spiflash_image(storage ../fatfs_image FLASH_IN_PROJECT)