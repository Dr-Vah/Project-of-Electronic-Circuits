# RemoteControl — 手机遥控小车固件

基于 **ESP-IDF 5.4.4 / ESP32-S3** 的遥控小车固件。小车支持蓝牙（BLE）与 Wi-Fi 两种控制方式，并带 USB 摄像头第一视角（FPV）、TFT「奶蛙」表情屏幕和语音控制。本工程为独立固件，控制页面直接内嵌在固件里，不访问互联网，也不需要电脑转发或安装 App。

## 功能特性

- **BLE 遥控**：通过 GATT 服务 `Omni-Remote-BLE` 接收驾驶指令（微信小程序为主要客户端）。
- **Wi-Fi 网页遥控**：小车开启热点 `Omni-Remote`（密码 `omni2026`），手机浏览器打开内嵌网页即可控制。
  - 触屏摇杆（HTTP 80）与手机体感倾斜（HTTPS 443，需信任本地 CA 证书）。
  - 三档限速、断联停车、单调序号与 300 ms 看门狗保护。
- **第一视角（FPV）**：USB UVC 摄像头取流，直接转发 MJPEG，通过 HTTP 81 / HTTPS 8443 给浏览器、TCP 8266 给小程序。
- **TFT 奶蛙表情**：ST7735S 128×160 屏幕上显示「奶蛙」双帧动画表情，随情绪自动切换或手动选择。
- **情绪系统**：眩晕、疲劳、闲置睡眠等状态驱动表情（不影响车速）。
- **语音控制**：通过 GPIO 读取语音识别模块，实现前进/后退/转向/停止。
- **控制保护**：驾驶令牌、220 ms 指令超时、300 ms 看门狗、STOP 全端开放、平移/旋转限速与缓变。

## 硬件与接线

| 部件 | 说明 |
| --- | --- |
| 主控 | ESP32-S3（16 MB PSRAM，Octo SPI Flash） |
| 电机 | 3 个全向轮（麦克纳姆）电机，见下方引脚表 |
| 屏幕 | ST7735S 128×160（显示奶蛙表情） |
| 摄像头 | USB UVC（MJPEG），D− 接 GPIO19、D+ 接 GPIO20，5V 供电并共地 |
| 语音模块 | PA4 → GPIO21（停止），PA0 → GPIO1、PA1 → GPIO2 |

### 电机引脚

| 电机 | PWM | IN1 | IN2 | 编码器 A/B |
| --- | --- | --- | --- | --- |
| A 右前 | 16 | 18 | 17 | 8 / 9 |
| B 后 | 4 | 6 | 5 | 15 / 7 |
| D 左前 | 14 | 12 | 13 | 10 / 11 |

### TFT 引脚

| TFT 接口 | ESP32-S3 |
| --- | --- |
| SDA / MOSI | GPIO42 |
| SCL / SCK | GPIO41 |
| DC | GPIO48 |
| RST | GPIO38 |
| CS | GND |

## 构建与烧录

在 ESP-IDF 环境中：

```powershell
# 首次构建（或更换电脑）先重新生成本地 TLS 证书
python setup_tls.py

idf.py build
idf.py -p COM5 flash monitor
```

`COM5` 为示例串口，请替换为实际开发板串口。烧录会替换板上当前应用；完整烧录参数由 `build/flash_args` 提供，不要只写应用 bin 到地址 0。

> 依赖说明：工程使用自定义分区表 `partitions.csv`、蓝牙、PSRAM、USB Host（UVC）等配置，均已写入 `sdkconfig.defaults`。

## 连接与使用

1. 烧录后复位小车，手机连接热点 **Omni-Remote**（密码 **omni2026**）。
2. 完整浏览器打开 **<http://192.168.4.1/**（HTTP> 摇杆，无需证书）。
3. 若要体感控制，打开 **<https://192.168.4.1/**（需先下载并信任小车证书> `/omni-root.cer`）。
4. 微信小程序：连接 BLE `Omni-Remote-BLE` 即可驾驶，连接小车热点后可看画面。
5. 语音控制：语音模块识别指令，手机接管或停车后需重新说出停止口令才能恢复语音驾驶。

## 目录结构

```
RemoteControl/
├── components/usb_host_uvc/   # Espressif UVC 2.5.2 组件（USB 摄像头）
├── main/
│   ├── main.c                 # 热点、HTTP/HTTPS 服务、控制任务、BLE 初始化
│   ├── car_control.c/.h       # 电机与 PID（复用自 IntegratedRobot）
│   ├── ble_remote.c/.h        # BLE GATT 服务
│   ├── ble_protocol.h         # BLE 指令解析
│   ├── remote_state.h         # 驾驶权 / 序号 / 超时 / 输入校验
│   ├── index.html             # 内嵌网页控制页（摇杆 + 体感）
│   ├── fpv.c/.h               # UVC 取流与 HTTP/HTTPS 视频
│   ├── fpv_tcp.c/.h           # TCP 视频（给小程序）
│   ├── tft_display.c/.h       # ST7735S 屏幕驱动
│   ├── naiwa_render.c/.h      # 奶蛙双帧动画与 RLE 解码
│   ├── mood.h                 # 情绪累积与自动表情规则
│   ├── voice_control.h        # 语音模块 GPIO 控制
│   └── local_tls.h            # 本地 HTTPS 配置
├── assets/naiwa/              # 奶蛙表情素材与生成提示
├── tests/                     # 状态机 / UI / 协议 / TLS 测试
├── tools/pack_naiwa.py        # 素材离线打包
├── setup_tls.py               # 本地证书生成
├── partitions.csv             # 自定义分区表
└── sdkconfig.defaults         # 目标芯片与功能默认配置
```

## 测试

```powershell
gcc -std=c11 -Wall -Wextra -Werror tests/test_state.c -o tests/test_state.exe ; .\tests\test_state.exe
node tests/test_ui.cjs
node tests/test_fpv_ui.cjs
python tests/test_tls.py
gcc -std=c11 -Wall -Wextra -Werror tests/test_mood.c -o tests/test_mood.exe ; .\tests\test_mood.exe
```
