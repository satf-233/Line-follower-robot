idf_component_register(SRCS "utils.c" "motor.c" "servo.c" "lcd.c" "follow_brain.c" "camera_audio.c" "image.c" "encoder.c" "start.c" "test_cam.c" "test_audio.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi led_strip fatfs esp_jpeg)

fatfs_create_spiflash_image(storage ../fatfs_image FLASH_IN_PROJECT)
