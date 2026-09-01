# Task 2 ：视觉巡线、避障与终点控制

这是一个基于 ESP-IDF 5.4 的 ESP32-S3 小车工程。小车使用 USB UVC 摄像头模拟四路巡线传感器，通过三轮全向底盘完成巡线、转弯搜索和超声避障，并在 ST7735S 屏幕上实时显示二值化图像及四个视觉通道的检测结果。

## 功能概览

- USB UVC 摄像头采集 MJPEG 图像；
- 图像灰度二值化，并将指定区域划分为四个巡线通道；
- 三采样多数滤波，抑制单帧误判；
- 三轮全向底盘编码器闭环 PID 控制；
- 正常巡线、轻微修正、外侧大幅修正和丢线搜索；
- 第一次持续丢线时固定向右搜索，用于通过第一个弯道；
- HC-SR04 超声测距和左移绕障；
- 避障完成后才启用终点检测；
- 识别到四通道全黑后继续直行，离开黑色标志并变为全白后停车；
- 串口持续输出通道、巡线状态、避障状态和超声距离。

## 运行流程

1. 初始化电机、编码器、TFT、USB 摄像头和超声传感器，随后保持停车。
2. 摄像头产生有效画面后，将图像检测结果映射为四个虚拟巡线通道。黑色为 `1`，白色为 `0`。
3. 中间通道检测到线时直行或小幅修正；外侧通道检测到线时进行较强的转向修正。
4. 第一次持续丢线超过 150 ms 后，车辆固定向右旋转搜索；以后丢线则按照最后一次看到线的方向搜索。
5. 超声距离不大于 5 cm 时进入避障：停车稳定、左移、前进绕过障碍，再右移寻找原巡线。
6. 右侧外通道重新找到线后认为避障完成，并启用终点检测。
7. 避障前出现四通道全黑不会停车。避障完成后，四通道全黑会进入终点直行状态；车辆保持直行，直到四通道全白后停车并结束主控制循环。

## 屏幕显示

ST7735S 分辨率为 128 × 160：

- 上方 120 行显示实时二值化摄像头画面和四个彩色检测框；
- 下方显示四个通道的暗色像素比例；
- `*` 表示该通道判定为黑色（逻辑 `1`），`-` 表示白色（逻辑 `0`）。

当前运行页面不显示超声距离；距离数据包含在串口 `ultrasonic` 和 `status` 日志中。

## 硬件连接

### USB 摄像头、TFT 和超声模块

| 模块信号 | ESP32-S3 引脚 | 说明 |
| --- | ---: | --- |
| USB UVC D- | GPIO19 | 摄像头 USB 数据线 |
| USB UVC D+ | GPIO20 | 摄像头 USB 数据线 |
| TFT MOSI / SDI | GPIO42 | SPI3 MOSI |
| TFT SCK | GPIO41 | SPI3 时钟 |
| TFT D/C | GPIO48 | 数据/命令选择 |
| TFT RST | GPIO38 | 屏幕复位 |
| TFT CS | GND | 驱动中片选固定为低电平 |
| HC-SR04 TRIG | GPIO21 | 超声触发输出 |
| HC-SR04 ECHO | GPIO47 | 必须先将 5 V ECHO 降压至 3.3 V |

### 电机和编码器

| 轮组 | PWM | IN1 | IN2 | 编码器 EA | 编码器 EB |
| --- | ---: | ---: | ---: | ---: | ---: |
| A（右前） | GPIO16 | GPIO18 | GPIO17 | GPIO8 | GPIO9 |
| B（后轮） | GPIO4 | GPIO6 | GPIO5 | GPIO15 | GPIO7 |
| D（左前） | GPIO14 | GPIO12 | GPIO13 | GPIO10 | GPIO11 |

所有模块必须共地。电机应使用合适的独立电源和驱动器，不要直接由 ESP32-S3 GPIO 驱动。HC-SR04 的 ECHO 为 5 V 信号，连接 GPIO47 前必须使用电阻分压或电平转换器。

## 软件环境与依赖

- ESP-IDF：`>= 5.4.0, < 5.5.0`
- 目标芯片：ESP32-S3
- PSRAM：Octal、80 MHz
- `espressif/esp_jpeg`：1.3.1
- `espressif/usb_host_uvc`：2.5.2

依赖声明位于 `main/idf_component.yml`。第一次构建时，ESP-IDF Component Manager 会下载缺少的组件。

## 构建、烧录和监视串口

在 ESP-IDF 终端中执行：

```powershell
Set-Location 'C:\Users\Cyan_\Desktop\THU\dianshe\vscode-esp32s3\esp-projects\task 2 ququ'
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换为开发板的实际串口，例如 `COM5`。只监视串口时使用：

```powershell
idf.py -p COM5 monitor
```

按 `Ctrl+]` 退出串口监视。

成功构建后，固件位于：

```text
build/task_1.bin
```

## 当前关键参数

| 参数 | 当前值 | 位置 |
| --- | ---: | --- |
| 主控制周期 | 20 ms | `main/main.c` |
| 状态日志周期 | 500 ms | `main/main.c` |
| 超声采样周期 | 50 ms | `main/main.c` |
| 正常前进速度 | 0.04 m/s | `main/main.c` |
| 丢线确认时间 | 150 ms | `main/main.c` |
| 搜索超时 | 9000 ms | `main/main.c` |
| 障碍触发距离 | 5 cm | `main/main.c` |
| 障碍清除距离 | 150 cm | `main/main.c` |
| 左右平移速度 | 0.06 m/s | `main/main.c` |
| 绕障前进时间 | 2400 ms | `main/main.c` |
| 图像二值化阈值 | 110 | `main/camera_line_sensor.c` |
| 通道黑色占比阈值 | 20% | `main/camera_line_sensor.c` |
| 编码器标定 | 6000 ticks/m | `main/car_control.h` |
| PID 参数 | Kp=2.0，Ki=4.0，Kd=0.0 | `main/car_control.h` |

赛道、光照、摄像头角度或车轮发生变化后，应重新标定图像阈值、ROI、速度、绕障时间和编码器参数。

## 主要文件

| 文件 | 作用 |
| --- | --- |
| `main/main.c` | 巡线、搜索、避障和终点状态机 |
| `main/camera_line_sensor.c` | UVC 采集、JPEG 解码、二值化和四通道分析 |
| `main/car_control.c` | 三轮运动学、电机 PWM、编码器测速和 PID |
| `main/ultrasonic_sensor.c` | HC-SR04 触发、回波计时和距离换算 |
| `main/tft_display.c` | ST7735S 初始化、图像和文字绘制 |

## 串口日志说明

- `sensor`：四通道电平、黑色判定、原始/滤波位图和位置误差；
- `state`：巡线状态切换；
- `ultrasonic`：测距值或无回波、超时、超量程原因；
- `status`：当前巡线状态、避障阶段、距离和丢线时间；
- `camera_line`：摄像头连接、MJPEG 模式和断开信息。

若摄像头尚未提供有效帧，小车会把通道视为全白并保持停车。若右移回线在 5 秒内未找到巡线，系统进入故障状态并停止电机。
