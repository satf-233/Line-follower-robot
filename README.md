# Line-follower robot

基于 ESP-IDF 的 ESP32-S3 循迹小车项目。

## TODO

- [x] WS2812 灯珠点亮示例（GPIO 38，RMT 驱动）
- [x] 电机可用性测试
- [ ] 封装移动相关函数
- [ ] 使用 LEDC 完成 PWM 信号的输出
- [ ] 确定电机控制相关的引脚
- [ ] 循迹传感器读取与循迹逻辑

## 硬件

- 芯片：ESP32-S3
- 灯珠：WS2812 × 1（GPIO 38）
- 电机驱动：有 TB6612FNG × 2 的驱动板
- 电机 × 3
- 摄像头 × 1
- ...

## 环境

- ESP-IDF v5.4.4
- 依赖：espressif/led_strip ^3.0.3

## 构建

> [!WARNING]
> 在构建与烧录前，务必保证pin.h与实际连接情况一致

> [!WARNING]
> 当前与电机控制相关的引脚未完全确定，因此代码中有部分缺少定义的情况，构建时会发生错误。可以注释掉相关内容以尝试构建。

```bash
idf.py build
```

## 烧录

> [!WARNING]
> 在构建与烧录前，务必保证pin.h与实际连接情况一致

（使用USB转串口时）
```bash
idf.py -p COM5 flash
```
