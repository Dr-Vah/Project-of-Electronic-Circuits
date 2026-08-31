# ESP32-S3 USB 摄像头实时显示工程

本工程通过 ESP32-S3 的 USB Host 接收 UVC 摄像头输出的 MJPEG 视频，解码为 RGB565 后实时显示到 128x160 ST7735S 屏幕。工程只负责摄像头采集、JPEG 解码和屏幕显示，不包含视觉巡线或电机控制。

运行状态可分为四层：

1. 串口出现 `UVC camera found`：供电、D+/D- 和 USB 枚举正常。
2. 串口出现 `profile accepted`：摄像头接受了某个 MJPEG 视频模式。
3. 屏幕先显示蓝色：ST7735S 初始化和 SPI 通信正常。
4. 每秒出现 `DISPLAY OK`：已经持续接收、解码并显示视频帧。

## 接线

## 四路巡线视觉调试（本次新增）

工程启动后会创建热点 `ESP32-Camera-Debug`（密码 `camera123`）。连接热点后，在浏览器打开
`http://192.168.4.1/`。页面显示摄像头画面的二值化结果，并将画面从左到右切为 ROI 1 至 ROI 4。

可在页面实时调整：

- `Threshold`：灰度小于等于该值的像素视为黑线；
- `ROI top/bottom`：参与判断的图像纵向范围，使用百分比，默认取下方 55%；
- `Active`：一个 ROI 中黑色像素比例达到该百分比时，显示为 `ON`。

每个 ROI 同时显示黑色像素数、比例、黑色像素水平质心和 ON/OFF。ESP32 使用同一组参数在
解码后的 RGB565 图像上计算这些数值，后续巡线 main 可将 ROI 1..4 的 ON/OFF 直接映射为原红外
模块 OUT1..OUT4（图像从左到右）。本阶段只做检测与可视化，不会驱动电机。

| 摄像头线 | ESP32-S3 | 说明 |
| --- | --- | --- |
| 5V | 开发板 5V 或独立稳压 5V | 不能接任何 GPIO；供电应能承受摄像头启动电流 |
| D- | GPIO19 | USB 原生差分线，尽量短，并和 D+ 绞合 |
| D+ | GPIO20 | USB 原生差分线，尽量短，并和 D- 绞合 |
| GND | GND | 若用独立 5V，必须和 ESP32-S3 共地 |

摄像头图片中插头从左到右的颜色标注应再次以实物丝印为准，不要只凭线色接电源。上电前用万用表确认 5V 与 GND 没有接反。

GPIO19/20 同时是 ESP32-S3 原生 USB-OTG 引脚，调试期间不能用该 USB 口烧写或看串口。请用开发板上连接 CP210x 的 UART USB 口烧写和监视日志。

### ST7735S 屏幕信号线

| 屏幕信号 | ESP32-S3 |
| --- | --- |
| SDA / MOSI | GPIO42 |
| SCL / SCLK | GPIO41 |
| DC | GPIO48 |
| RST | GPIO38 |

屏幕的电源、地和背光请按模块标称电压连接并与 ESP32-S3 共地。当前驱动不使用 CS 引脚。

## 构建和烧写

在 ESP-IDF 5.4.4 PowerShell 环境运行：

```powershell
cd "D:\study\Project of Electronic Circuits\esp-projects\camera-uvc-test"
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

首次构建需要联网下载 Espressif 的 `usb_host_uvc` 组件。将 `COMx` 换成 CP210x 对应端口。

## 模组存储配置

上传的资料是 ESP32-S3-WROOM-2，该系列带 8 MB 或 16 MB Octal PSRAM，工程已经开启 Octal PSRAM。若模组金属屏蔽罩上的完整型号不是 `ESP32-S3-WROOM-2-*`，先不要烧写，把完整型号拍照确认后再调整 `sdkconfig.defaults`。

## 日志判读

- 一直停在 `waiting for camera`：先查 5V、电流能力、共地、D+/D- 是否接反。
- 找到 USB 设备但提示 `no UVC camera`：设备没有暴露 UVC 接口，或复合设备枚举异常。
- 所有 `profile rejected`：查看此前打印的描述符，把摄像头明确支持的 MJPEG 分辨率/帧率加入 `s_profiles`。
- `start streaming failed`：通常是所选模式带宽过大；优先降低分辨率或帧率。
- 屏幕未短暂显示蓝色：检查屏幕供电、共地和 SPI 信号线。
- 有视频帧但 `decode failures` 持续增加：检查差分线长度、接触和 5V 供电稳定性。
- 有 `DISPLAY OK` 但屏幕不更新：检查 SDA、SCL、DC、RST 接线以及屏幕控制器是否为 ST7735S。

图片标称的 1280x720 是传感器/摄像头能力，不代表 ESP32-S3 的 USB Full-Speed 链路适合直接接收该模式。首轮测试从 320x240 MJPEG 开始，确认链路后再逐级提高。
