# BallDet 白球检测

`host_test.cpp` 已在 `dianshe` 目录中的 6 张实拍图上逐张验证，6/6 均将真实白球选为最佳目标。相同的整数算法现已迁移到 `main/ball_detector.c`，并接入 USB UVC 摄像头、JPEG 解码、三帧稳定确认和 LCD 标记显示流程。

## 算法

1. 将输入等比例降采样到 320 像素宽，降低噪声和运算量。
2. 计算灰度、RGB 色差和 Sobel 梯度。
3. 对半径 8–14 像素的圆使用 32 个固定采样点（整数查表，无三角函数）。
4. 检查圆周梯度是否沿半径方向、是否覆盖四个象限。
5. 利用白球特征过滤干扰：上缘外亮内暗、下缘外暗内亮、球内低色差、球外是明亮且平滑的天花板。
6. 按圆周支持度和球体明暗梯度评分，只输出最高分目标。

程序还会自动识别并裁掉网页截图顶部的深色标题区域。ESP32 摄像头原始帧不需要这一步。

## 编译

在 MinGW PowerShell 中运行：

```powershell
cd C:\Users\Cyan_\Desktop\THU\dianshe\vscode-esp32s3\esp-projects\BallDet
g++ -std=c++17 -O2 -municode host_test.cpp -o ball_host_test_v2.exe -lgdiplus -lole32 -luuid
```

## 测试单张图片

```powershell
.\ball_host_test_v2.exe "C:\Users\Cyan_\Desktop\THU\dianshe\C50FDEB35030FA61D240834B2112148D.png"
```

结果写入 `output_v2`：

- `*_detected.png`：绿色圆为最终检测结果，红点为圆心。
- `*_small.png`：算法实际处理的 320 像素宽灰度图。

控制台同时输出圆心、半径、总分、圆周覆盖率、上下明暗差、背景亮度、色差和背景纹理，方便用新增图片继续调参。

## 当前适用条件

当前阈值针对本次数据集：白球位于明亮、低饱和度且较平滑的天花板背景，原图中的球半径约为图像宽度的 2.5%–3.5%。若距离范围或场地照明变化很大，应补充含球和不含球样本后再调整半径、背景亮度及色差阈值。

## ESP32-S3 构建

```powershell
idf.py -B build_board build
```

固件位于 `build_board/balldet.bin`。板端会优先选择宽度不低于 320 的最小 MJPEG 模式，按 JPEG 实际尺寸在 PSRAM 中复用帧缓冲；检测时缩到 320 像素宽，连续 3 帧位置接近才在 LCD 上画绿色圆和红色圆心，并输出 `BALL stable` 日志。

当前已经通过 ESP-IDF 5.4.4 完整构建，但尚未执行烧录。下一阶段是连接开发板后烧录并根据串口中的检测耗时、PSRAM 余量和真实画面做阈值微调。
