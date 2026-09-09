idf_component_register(SRCS
"WIFI.c"
"image.c"
"BLE.c"
"utils.c"
"motor.c"
"lcd.c"
"avoid.c"
"camera_audio.c"
"start.c"
"test_WIFI.c"
"test_thres.c"
"test_mask.c"
"ball.c"
"test_ball.c"
"mask_stream.c"
"test_ball_noWIFI.c"

INCLUDE_DIRS "."
REQUIRES
esp_driver_gpio
esp_driver_ledc
esp_driver_spi
esp_timer
led_strip
fatfs
esp_jpeg
bt nvs_flash
esp_wifi
esp_http_server
esp_netif
esp_event)

fatfs_create_spiflash_image(storage ../fatfs_image FLASH_IN_PROJECT)
