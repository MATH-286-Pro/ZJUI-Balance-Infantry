# 使用说明

- `tasks.json` 文件中有 2 个任务：
    - `buildEmbeddedTargets` 任务作为 `lanuch.json` 中的 Prelaunchtask，在调试前先行编译文件，更方便
    - `Download to STM` 任务用于烧录程序，使用方法为在终端输入 `openocd -c 烧录.hex文件`


- 开启的外设：
  - UART1 用于宇树A1电机测试(未成功)
  - UART6 用于测试串口波特率极限
  - UART3 用于遥控器接收
  - I2C2 用于OLED
  - TIM4 PWM蜂鸣器 (挂载于APB2外设时钟)
  - CAN1 控制小米电机 (挂载于APB?外设时钟)

- 测试日志
  - 2024.3.18 使用示波器测试 UART1 TX GND
    - 注意: 示波器探针不要接到RX上，因为没有人给UART1发送数据，所以不会有任何示波器数据
    - 结论：
      - 可能是是转换器的问题
      - 宇树电机不需要通电也可以传递 RS485 信号，可以不开电查看电信号
      - A1_motor_speed_contrl 使用 UART1，并且确实在发送信号 

      | 设备 | 描述 |
      | --- | --- |
      | UART1 | 示波器有信号(4M 4.8M) |
      | TTL转RS485 | 示波器无信号 |
      | A1电机 | 示波器有波形 |