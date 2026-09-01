# Task 1：巡线、避障与数据显示

这是一个可独立构建的 ESP-IDF 工程，目标芯片为 ESP32-S3，集成以下功能：

- 四路红外循线与丢线搜索；
- 三轮全向底盘编码器闭环 PID 控制；
- HC-SR04 超声测距和左移绕障；
- ST7735S 屏幕显示 A、B、D 三轮实测 RPM 与超声距离；
- 障碍物清除后继续左移 200 ms，以增加侧向间距。

## 构建与烧录

在 ESP-IDF 终端进入本目录：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换为开发板实际串口号。

## 主要硬件连接

| 模块 | 引脚 |
| --- | --- |
| TFT MOSI/SDI | GPIO20 |
| TFT SCK | GPIO19 |
| TFT D/C | GPIO48 |
| TFT RST | GPIO38 |
| TFT CS | GND |
| 超声 TRIG | GPIO21 |
| 超声 ECHO | GPIO47（必须降压至 3.3 V） |
| 红外 OUT1..OUT4 | GPIO42、41、40、39 |

电机、编码器引脚见 `main/car_control.h` 中的
`CAR_CONTROL_DEFAULT_CONFIG()`。

## 关键参数

- 轮径：57 mm；
- TFT 刷新周期：200 ms；
- 超声采样周期：50 ms；
- 额外左移时间：200 ms；
- 编码器标定值：8900 ticks/m。

速度、避障阈值和状态机参数集中在 `main/main.c`，PID 参数位于
`main/car_control.h`。
