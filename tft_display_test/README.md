# TFT standalone test

这个工程只测试 TFT 显示，不使用电机、编码器、红外、加速度传感器或超声波模块。

测试程序每秒更新一次固定的 RPM 和距离数值，用于确认：

- ST7735S 128x160 屏幕的方向和可见区域；
- 文字是否正常显示；
- 局部刷新是否会闪烁或产生乱码。

## 构建

在 ESP-IDF 环境中执行：

```text
idf.py set-target esp32s3
idf.py build
```

## 烧录和查看日志

```text
idf.py -p COM5 flash monitor
```

也可以使用已经生成的 `build` 目录直接烧录。

正常运行时，屏幕上的 A/B/D 三个 RPM 和距离值会每秒变化一次，串口会输出对应数值。
