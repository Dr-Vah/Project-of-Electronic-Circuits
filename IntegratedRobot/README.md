# IntegratedRobot

ESP32-S3 三轮全向车完整任务工程。程序先完成巡线、首个大弯和超声避障，越过终点后停车 5 秒，再使用同一个 UVC 摄像头寻找球、分配黑区并依次运输白球和橙球。

## 代码来源

当前工程由两套代码整合而成。

| 功能 | 来源 | 当前状态 |
| --- | --- | --- |
| 巡线、弯道搜索、超声避障、终点判定 | GitHub `dev/task 2 fixed` | 作为主任务基础，已针对整合后的时序和实车表现修改 |
| 电机闭环、编码器和三轮全向运动 | `dev/task 2 fixed` | `car_control.c/.h` 原样采用 |
| HC-SR04 底层驱动 | `dev/task 2 fixed` | `ultrasonic_sensor.c/.h` 原样采用；清空判定在 `main.c` 中修改 |
| TFT 驱动 | `dev/task 2 fixed` | 保留巡线调试显示，新增运球阶段全彩画面显示 |
| 白球、橙球和黑区识别 | 用户原 `WhiteBallDetection` | 识别实现原样采用 |
| 两阶段运球状态机 | 用户原 `WhiteBallDetection` | 保留原状态机，修改横移方式并复用已初始化的底盘 |
| 相机与底盘坐标标定 | 用户原 `WhiteBallDetection` | 保留原推球轴和 180° 坐标修正 |

独立的 `dev/BallDet v2` 检测器和结果适配器已经从工程中移除，不参与当前编译。

需要注意：`WhiteBallDetection` 自带的 `ball_detector.c/.h` 在其源码注释中标明最初来自旧版 `dev/BallDet/main` commit `9f08d21`。当前工程复制的是 `WhiteBallDetection` 中保存的这份依赖，并非此前接入后又移除的 BallDet v2 流水线。

## 任务流程

1. 初始化底盘、电机编码器、TFT、UVC 摄像头和超声波传感器。
2. 摄像头画面转换成四路虚拟红外，并计算连续的黑线质心误差。
3. 正常巡线；第一次持续丢线时执行指定方向的大弯搜索。
4. 超声检测到近距离障碍物后：
   1. 停车 150 ms；
   2. 向左横移，直到距离不小于 150 cm 或返回 `ULTRASONIC_OUT_OF_RANGE`；
   3. 额外向左横移 200 ms；
   4. 向前绕过障碍物 2400 ms；
   5. 向右横移，直到摄像头重新发现靠近中心的黑线。
5. 避障完成后启用终点检测。进入全黑终点区时继续直行，离开终点区、四路全白后停车。
6. 保持停车 5 秒。
7. 不重启 USB、不重复初始化底盘，原相机任务切换到 `WhiteBallDetection`。
8. 识别白球、橙球和各自对应的黑区，并提前锁定黑区归属。
9. 先把白球运输到左侧黑区，再寻找橙球并运输到右侧黑区。

巡线结束前只有 `main.c` 控制电机。等待 5 秒后才创建 `ball_transport_controller` 任务，因此两个阶段不会同时写入底盘速度。

## 整合后做过的修改

### 1. 共用摄像头和底盘

- UVC 摄像头只初始化一次，使用 GPIO19/20 的 USB-OTG 接口。
- 巡线阶段和运球阶段共用 `camera_line_sensor.c` 中的 JPEG 解码任务。
- 阶段切换时只改变图像处理模式，不关闭和重新打开摄像头。
- 底盘只在上电时初始化一次；从运球控制器中删除了重复的 `car_control_init()`。

### 2. 巡线视觉

- 巡线阶段恢复 `task 2 fixed` 的固定 `JPEG_IMAGE_SCALE_1_4`，避免为球识别提高分辨率后增加巡线延迟。
- 运球阶段才根据相机模式自动选择 1/2、1/4 或 1/8 解码比例，最大处理尺寸为 240×160。
- 二值化亮度阈值从 110 提高到 125。
- 巡线 ROI 保持画面顶部 0%～8%、宽度 30%、中心向左偏移 7%。
- 原始巡线画面顶部对应近车区域；近端行使用 3 倍权重，向远端线性降低到 1 倍。四路黑色占比和连续质心都使用该权重。
- TFT 巡线调试画面限制为约 10 FPS，视觉状态仍逐帧更新。

### 3. 弯道与丢线处理

- 丢线后立即停车，再完成 150 ms 的丢线确认，不再沿用上一条前进或弧线命令。
- 第一个大弯的强制右转保护时间从 1000 ms 缩短到 600 ms。
- 中间区域重新看到黑线后，需要连续两个不同的相机帧确认，不能把同一缓存帧重复计数。
- 搜索转速保持 0.25 rad/s，普通巡线最大转向角速度保持 0.20 rad/s。

### 4. 超声避障

- 触发距离为 6.5 cm，确认后先停车 150 ms。
- 左移速度为 0.06 m/s。
- 有效距离不小于 150 cm 时判定障碍物已经离开。
- `ULTRASONIC_OUT_OF_RANGE` 也直接作为大于 150 cm 处理。
- `ULTRASONIC_WAIT_RISE_TIMEOUT` 和 `ULTRASONIC_PULSE_TIMEOUT` 仍视为无效数据，不触发清空。
- 绕障前进速度为 0.06 m/s，持续 2400 ms；返回黑线时以 0.04 m/s 向右横移。

### 5. 球和黑区识别

- 白球使用 `white_ball_detector.cpp` 和其内部依赖 `ball_detector.c`。
- 橙球使用 `orange_ball_detector.c`。
- 黑区使用 `black_target_detector.c`。
- 第一阶段同时检测白球和橙球，分别保存球的位置锚点，并为两个黑区建立独立标签。
- 在白球对应黑区和橙球对应黑区都确认前，不允许开始第一段运输，避免车辆转向后重新分配黑区。
- 图像与检测结果按原 `WhiteBallDetection` 流程旋转 180° 后送入运球控制器。

### 6. 运球控制

- 运球状态机取自用户原 `WhiteBallDetection/main/ball_transport_controller.c`。
- 保留寻找、转向、接近、三点一线、推球、后退和切换球色等原状态。
- 三点一线前的横向校准由 60 mm/s、100 ms 的启停脉冲改为低速连续横移。
- 连续横移按误差比例控制，增益为 0.045，最大横移速度为 0.020 m/s。
- 进入对齐容差后停车并等待稳定帧，不再出现反复踢动造成的左右颤动。

## 当前编译文件

`main/CMakeLists.txt` 当前编译以下文件：

| 文件 | 作用 | 来源与改动 |
| --- | --- | --- |
| `main.c` | 总任务、巡线、弯道、避障、终点和阶段切换 | 以 `task 2 fixed` 为基础，加入上述控制修改和 5 秒切换 |
| `camera_line_sensor.c/.h` | UVC、JPEG、巡线视觉、球识别调度 | 以 `task 2 fixed` 为基础，整合 `WhiteBallDetection` |
| `car_control.c/.h` | 三轮运动学、电机 PWM、编码器、闭环控制 | `task 2 fixed` 原样采用 |
| `ultrasonic_sensor.c/.h` | HC-SR04 TRIG/ECHO 测距 | `task 2 fixed` 原样采用 |
| `tft_display.c/.h` | 巡线调试、速度距离和相机画面 | `task 2 fixed`，新增全彩相机帧接口 |
| `white_ball_detector.cpp/.h` | 白球检测和跟踪 | `WhiteBallDetection` 原样采用 |
| `ball_detector.c/.h` | 白球圆形梯度检测底层 | `WhiteBallDetection` 原样采用 |
| `orange_ball_detector.c/.h` | 橙球连通域检测和跟踪 | `WhiteBallDetection` 原样采用 |
| `black_target_detector.c/.h` | 黑区检测、分配和锁定 | `WhiteBallDetection` 原样采用 |
| `ball_transport_controller.c/.h` | 两阶段运球状态机 | `WhiteBallDetection`，改为连续横移并删除重复底盘初始化 |
| `chassis_camera_geometry.h` | 相机中心带和推球轴标定 | `WhiteBallDetection`，补充整合坐标说明 |

## 关键硬件连接

### UVC 摄像头

- USB D-：GPIO19
- USB D+：GPIO20

### HC-SR04

- TRIG：GPIO21
- ECHO：GPIO47
- HC-SR04 ECHO 是 5 V，接入 ESP32-S3 前必须分压或电平转换到 3.3 V。

### TFT

- SDI/MOSI：GPIO42
- SCK：GPIO41
- DC：GPIO48
- RST：GPIO38
- CS：硬件固定低电平

### 三个电机

电机 PWM、方向和编码器 GPIO 定义在 `main/car_control.h` 的 `CAR_CONTROL_DEFAULT_CONFIG()` 中。整合工程保持 `task 2 fixed` 的原值。

## 构建与烧录

需要 ESP-IDF 5.4.x。依赖固定为：

- `espressif/esp_jpeg 1.3.1`
- `espressif/usb_host_uvc 2.5.2`

在 ESP-IDF PowerShell 中执行：

```powershell
cd "D:\study\Project of Electronic Circuits\IntegratedRobot"
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

将 `COM5` 替换为设备管理器中实际显示的串口。只烧录、不打开串口监视器：

```powershell
idf.py -p COM5 flash
```

退出串口监视器使用 `Ctrl+]`。

工程已使用 ESP-IDF 5.4.4 完成编译验证。当前固件约 366 KB，单应用分区为 1 MB，剩余约 65%，固件 Flash 占用不高。

编译产物位于：

```text
build-verify/IntegratedRobot.bin
```

## 实车调试重点

- 上电后确认串口打印出的 UVC 模式和帧率。
- 巡线时观察 `black`、`pattern`、`error` 和状态变化。
- 避障时以 `avoid=...` 为实际电机控制状态；同时出现的 `state=CORRECT_LEFT` 等只是暂停前保存的巡线状态。
- `ULTRASONIC_OUT_OF_RANGE` 当前会立即结束左移清空判断；无回波和 ECHO 高电平超时不会。
- 球识别前确认 PSRAM 初始化成功，否则相机缓冲区和检测器无法创建。
- 烧录前再次核对电机、超声和 TFT GPIO 与实车接线一致。
