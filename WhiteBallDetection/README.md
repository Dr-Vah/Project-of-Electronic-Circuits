# 白球识别程序

这是一个独立的 RGB565 白球与黑色停止区视觉调试程序，不包含巡线、超声波、
电机控制或运球状态机。

调用方式：

```c
white_ball_config_t config;
white_ball_default_config(&config);

/* detector 约占 115 KB，建议放在静态区或 PSRAM，不要放在任务栈。 */
static white_ball_detector_t detector;
white_ball_detector_init(&detector, &config);

white_ball_result_t ball;
white_ball_detect_rgb565(&detector, rgb565_frame,
                         frame_width, frame_height, &ball);

black_target_result_t target = {0};
/* 白球确认后调用 black_target_detect_rgb565() 填充 target。 */

/* 每收到并解码一帧摄像头图像后立即刷新屏幕。 */
white_ball_display_show_rgb565(rgb565_frame, frame_width, frame_height,
                               config.rgb565_byte_swapped, &ball, &target);
```

识别步骤为：直接高亮或阴影内中灰候选提取、八邻域连通域、局部亮度差、
16方向圆周边缘、附近纸面检查和四次有效观测确认。中间允许短暂漏掉1～2帧，
确认后也会短时保持最近可靠位置，避免检测框闪烁。阴影内候选还必须满足严格的
圆度、填充率、直径和背景亮度条件。默认支持最大 `240 x 160` 图像，
并仅搜索完成180度方向校正后画面的指定区域，减少房间背景干扰。

黑色停止区使用暗色连通域识别。只有白球确认后才寻找黑区，并选择位于白球
前方、横向最接近且前后距离合理的候选，避免把门后或家具阴影当成停止区。

该目录现在是独立的 ESP-IDF 5.4.x 工程。`main/main.c` 只负责USB UVC摄像头
取流、JPEG解码、白球/黑区识别、屏幕预览、Wi-Fi回传和串口调试，不包含巡线
或电机控制。

现场优先调整：

- `minimum_luma`：默认210；球的高亮区域必须达到该亮度；
- `minimum_dim_luma`：默认100；阴影模式允许的最低球体亮度；
- `local_highlight_difference`：默认6；阴影中的球心需比对称外围纸面至少亮6；
- `maximum_chroma`：默认64；用于排除橙色灯等强彩色物体；
- `minimum_local_luma_contrast`：球必须比周围纸面至少亮10；
- `minimum_confidence`：总评分门槛，默认0.50；
- `minimum_area_px`：按白球在实际画面中的最小像素面积设置。

## 实时屏幕预览

初始化检测器时同时初始化屏幕：

```c
#include "white_ball_display.h"

ESP_ERROR_CHECK(white_ball_display_init());
```

屏幕使用项目现有的 128x160 ST7735S 接线：

| ST7735S | ESP32-S3 |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| CS | GND（必须接低，不能悬空） |
| SDA / MOSI | GPIO42 |
| SCL / SCLK | GPIO41 |
| DC | GPIO48 |
| RST | GPIO38 |

摄像头彩色画面保持宽高比并居中显示。绿色框和青色十字标记白球，品红框标记
选中的黑色停止区。两条黄色竖线标出标定后的车体中心带，红色竖线标出车体
直行轴，并非摄像头光学中心。原始网页画面的中心带是 `38%～48%`、轴线是
`43%`；TFT画面经过180度旋转，对应为 `52%～62%` 和 `57%`。
启动后屏幕会先显示蓝色自检画面；摄像头成功输出第一帧后才切换成实时画面。
如果一直停留在蓝色，表示屏幕正常，但USB摄像头尚未产生可解码的视频帧。

## Wi-Fi画面回传

固件启动后会建立热点 `ESP32-Camera-Debug`，密码为 `camera123`。电脑或手机
连接该热点后，在浏览器打开 `http://192.168.4.1/` 即可查看摄像头实时画面。
网页回传摄像头原始JPEG画面，并叠加绿色白球框、品红停止区框、黄色车体中心
带边界和红色直行轴；顶部同步显示帧数及两类检测状态。网页检测框坐标会自动
转换回原始画面方向。

## 构建与烧录

在 ESP-IDF 5.4 命令行中执行：

```powershell
cd "D:\study\Project of Electronic Circuits\WhiteBallDetection"
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

把 `COM5` 换成开发板实际串口。退出串口监视器使用 `Ctrl+]`。

首次构建时组件管理器需要联网下载 `usb_host_uvc` 和 `esp_jpeg`。烧录时可暂时
拔下USB摄像头，只保留ESP32-S3的下载串口；烧录完成后再给摄像头接入GPIO19/20、
5V和共地。

