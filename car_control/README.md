# STM32F103 小车总控工程

本工程将 WS2812 跑马灯和 PWM 电机驱动合并到同一个 Keil 项目中。`USER/main.c` 只负责安排功能执行顺序；每个硬件功能在 `QST_HARDWARE` 下各自维护驱动文件。

## 当前默认行为

前后两条 WS2812 灯链持续显示跑马灯。电机同时按以下顺序循环：

1. 前进 `FORWARD_TIME_MS`；
2. 停车 `DIRECTION_PAUSE_MS`；
3. 后退 `REVERSE_TIME_MS`；
4. 停车 `DIRECTION_PAUSE_MS`。

## 后续修改位置

| 想实现的内容 | 修改位置 |
| --- | --- |
| 调整前进/后退速度和时间 | `USER/main.c` 顶部的 `FORWARD_*`、`REVERSE_*` 宏 |
| 改变功能执行顺序 | `USER/main.c` 的 `while (1)` 循环 |
| 修改电机 PWM、方向或引脚 | `QST_HARDWARE/motor/motor.c` |
| 修改跑马灯颜色、速度或效果 | `QST_HARDWARE/colorful_led/colorful_led.c` |
| 新增传感器、按键等功能 | 在 `QST_HARDWARE` 新建对应 `.c/.h`，再在 `main.c` 调用 |

## 使用方法

在 Keil MDK 打开 `USER/car_control.uvprojx`，执行 **Rebuild All** 后下载。工程目标为 STM32F103C8T6。
