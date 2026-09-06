# ================================================================
# 摄像头循线构建集（红外循线套已移出：follow_brain.c / avoid.c / start.c）
#   test_follow.c 为 app_main 入口占位文件：
#   按 image.h 的流程替换其内容 ——
#     motor_init(); cam_init(NULL); cam_start_stream();
#     image_init(); while(1){ image_follow(); }
#   （写完把本注释删掉即可）
# 无 storage 分区：不使用 fatfs 烧录镜像（camera_audio.c 仍需要 fatfs 头文件/链接库编译）
# BLE 已移出（BT 静态占用近 1MB，放不下 singleapp 的 1MB app 分区；要恢复推流调试时
#   把 BLE.c 加回 SRCS，REQUIRES 补回 bt nvs_flash，sdkconfig.defaults 已有 BT 配置）
# ================================================================
idf_component_register(SRCS
"image.c"
"test_follow.c"
"test_ble_cam.c"
"BLE.c"
"utils.c"
"motor.c"
"lcd.c"
"avoid.c"
"camera_audio.c"
"start.c"
"test_cam.c"
"test_audio.c"
"test_avoid.c"

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