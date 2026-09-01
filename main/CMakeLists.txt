<<<<<<< HEAD
idf_component_register(SRCS "test_follow.c" "utils.c" "motor.c" "servo.c" "lcd.c" "follow_brain.c" "avoid.c" "camera_audio.c" "encoder.c" "start.c" "test_cam.c" "test_audio.c"
=======
idf_component_register(SRCS "test_follow.c" "test_ble_cam.c" "BLE.c" "utils.c" "motor.c" "servo.c" "lcd.c" "follow_brain.c" "avoid.c" "camera_audio.c" "image.c" "encoder.c" "start.c" "test_cam.c" "test_audio.c"
>>>>>>> 63765a10bd5850d70190c44f78dbd81bfbddc0f5
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_timer led_strip fatfs esp_jpeg bt nvs_flash)

fatfs_create_spiflash_image(storage ../fatfs_image FLASH_IN_PROJECT)
