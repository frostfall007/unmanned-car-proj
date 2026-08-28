# STM32F103 小车 PWM 前进/后退

独立 Keil 工程：小车以 60% PWM 占空比前进 2 秒，停车 0.2 秒后后退 2 秒，再停车 0.2 秒，持续循环。串口 1（PA9/PA10，115200 baud）会打印当前方向。

## 原理图对应关系

| 功能 | STM32F103C8T6 引脚 | 原理图网络 |
| --- | --- | --- |
| 右电机 PWM | PB6 / TIM4_CH1 | R-IB |
| 左电机 PWM | PB7 / TIM4_CH2 | L-IA |
| 右电机方向 | PB13 | R-IA |
| 左电机方向 | PB14 | L-IB |

TIM4 配置为 18 kHz PWM，避免常见的电机可闻噪声。方向定义与参考工程 `Set_Pwm()` 的正负方向一致。

## 使用方法

1. 在 Keil MDK 中打开 `USER/motor_pwm_forward_reverse.uvprojx`。
2. 执行 **Rebuild All**，再下载到 STM32F103C8T6。
3. 小车应重复执行“前进 2 秒 → 停 0.2 秒 → 后退 2 秒 → 停 0.2 秒”。

电机参数均在 `USER/main.c` 顶部独立配置：`FORWARD_DUTY_PERCENT`、`REVERSE_DUTY_PERCENT`（速度，0 到 100），以及 `FORWARD_TIME_MS`、`REVERSE_TIME_MS`（时间，单位毫秒）。若实际前进/后退方向与车身相反，交换 `motor_forward()` 和 `motor_reverse()` 中的 `GPIO_ResetBits`、`GPIO_SetBits` 即可；这是由两个减速电机的实际接线方向决定的。
