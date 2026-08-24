# Line-follower robot

基于 ESP-IDF 的 ESP32-S3 循迹小车项目。

## 当前进展

- [x] WS2812 灯珠点亮示例（GPIO 38，RMT 驱动）
- [ ] TB6612 双路电机驱动（正反转 / PWM 调速）
- [ ] 循迹传感器读取与循迹逻辑

## 硬件

- 芯片：ESP32-S3
- 灯珠：WS2812 × 1（GPIO 38）
- 电机驱动：TB6612FNG（双路 H 桥，含编码器接口）

## 环境

- ESP-IDF v5.4.4
- 依赖：espressif/led_strip ^3.0.3

## 构建

```bash
idf.py build
```

## 烧录

```bash
idf.py -p COM9 flash
```
